#!/usr/bin/env python3

import pulp
import numpy as np
import json
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
LEGO_CONFIG_DIR = REPO_ROOT / 'config/lego_tasks'

class TaskAssignmentSolver:
    def __init__(self):
        self.inf = 1000000
        self.lambda_balance = 1000  # Weight factor for balancing
        self.block_names = []
        self.num_offsets = 7
        self.manifest = {}

    def load_matrix(self, out_dir):
        manifest_path = Path(out_dir) / 'robots_manifest.json'
        if not manifest_path.exists():
            raise RuntimeError(f"Robot manifest not found at {manifest_path}")
        with open(manifest_path, 'r') as f:
            self.manifest = json.load(f)
        robots = self.manifest.get('robots', [])
        if not robots:
            raise RuntimeError("Robot manifest does not list any robots")

        self.block_names = []
        self.block_reusable = []
        num_tasks = self.manifest.get('num_tasks', 0)
        num_grasps = self.manifest.get('num_grasps', self.num_offsets * 4)
        cost_tensors = []
        support_tensors = []
        blocks_per_row = None

        for robot in robots:
            cost_file = Path(out_dir) / robot.get('cost_matrix', '')
            if not cost_file.exists():
                raise RuntimeError(f"Missing cost matrix for robot {robot.get('id')}: {cost_file}")
            with open(cost_file, 'r') as f:
                header = f.readline().strip()
            tokens = [tok for tok in header.split(',') if tok]
            if blocks_per_row is None:
                blocks_per_row = tokens
                for token in tokens:
                    if '@' not in token:
                        continue
                    block_name, _ = token.split('@')
                    if block_name not in self.block_names:
                        self.block_names.append(block_name)
                        self.block_reusable.append("station" in block_name)
            num_blocks = len(self.block_names)

            cost_matrix = np.genfromtxt(cost_file, delimiter=',', skip_header=1)
            if cost_matrix.ndim == 1:
                cost_matrix = cost_matrix.reshape(1, -1)
            cost_matrix = cost_matrix[:, :len(tokens)]
            if num_tasks <= 0:
                num_tasks = cost_matrix.shape[0]
            cost_matrix = np.reshape(cost_matrix, (num_tasks, num_blocks, num_grasps))
            cost_tensors.append(cost_matrix)

            support_file = Path(out_dir) / robot.get('support_matrix', '')
            support_matrix = np.genfromtxt(support_file, delimiter=',', skip_header=1)
            if support_matrix.ndim == 1:
                support_matrix = support_matrix.reshape(1, -1)
            support_matrix = support_matrix[:, :num_grasps]
            support_tensors.append(support_matrix)

        # read delta_matrix
        delta_matrix = np.genfromtxt(Path(out_dir) / 'delta_matrix.csv', delimiter=',', skip_header=1)
        if delta_matrix.ndim == 1:
            delta_matrix = delta_matrix.reshape(1, -1)
        delta_matrix = delta_matrix[:, :-1]

        # read s_j
        s_j = np.genfromtxt(Path(out_dir) / 'support_req.csv', skip_header=1, delimiter=',')
        if s_j.ndim == 1:
            s_j = s_j.reshape(1, -1)
        # drop trailing empty column if present
        s_j = s_j[:, :len(robots)]

        # read precedence matrix
        precedence_matrix = np.genfromtxt(Path(out_dir) / 'precedence.csv', delimiter=',')
        if np.size(precedence_matrix) == 0:
            precedence_matrix = np.zeros((0, 2))
        else:
            precedence_matrix = np.atleast_2d(precedence_matrix)[:, :2]

        cost_matrix = np.array(cost_tensors)
        s = np.array(support_tensors)
        s_j = np.transpose(s_j)

        return cost_matrix, delta_matrix, s, s_j, precedence_matrix

    def solve_problem(self, n, m, p, q, c, s, delta, s_j, precedence_constraints):
        """
        n: number of robots
        m: number of tasks
        p: number of blocks
        q: number of grasp sides
        c: cost matrix
        s: support matrix
        delta: delta matrix (whether block type is compatible with task)
        s_j: support required matrix
        precedence_constraints: list of tuples (t1, t2) where t1 must be used before t2
        """

        ts = time.time()
        if (n > m):
            print("Number of tasks must be greater than or equal to the number of robots")
            return None

        # Define the problem
        self.prob = pulp.LpProblem("Task_Assignment", pulp.LpMinimize)

        # Define the variables
        x = pulp.LpVariable.dicts("x", (range(n), range(m), range(p), range(q)), cat='Binary')
        y = pulp.LpVariable.dicts("y", (range(n), range(m), range(q)), cat='Binary')
        z = pulp.LpVariable.dicts("z", (range(n), range(m-n+1)), lowBound=0, cat='Integer')
        z_max = pulp.LpVariable.dicts("z_max", (range(m-n+1)), lowBound=0, cat='Integer')
        z_min = pulp.LpVariable.dicts("z_min", (range(m-n+1)), lowBound=0, cat='Integer')

        # Objective function
        self.prob += pulp.lpSum(c[i][j][t][g] * x[i][j][t][g] for i in range(n) for j in range(m) for t in range(p) for g in range(q)) + \
                    pulp.lpSum(s[i][j][g] * y[i][j][g] for i in range(n) for j in range(m) for g in range(q)) + \
                    self.lambda_balance * pulp.lpSum(z_max[k] - z_min[k] for k in range(m-n+1))

        # Constraints
        # Each task has one robot and one block
        for j in range(m):
            self.prob += pulp.lpSum(x[i][j][t][g] for i in range(n) for t in range(p) for g in range(q)) == 1

        # A robot cannot be assigned to both pick-and-place and support tasks for the same task
        for i in range(n):
            for j in range(m):
                self.prob += pulp.lpSum(x[i][j][t][g] for t in range(p) for g in range(q)) + pulp.lpSum(y[i][j][g] for g in range(q)) <= 1

        # If a task needs support, a robot is assigned to support
        for j in range(m):
            self.prob += pulp.lpSum(y[i][j][g] for i in range(n) for g in range(q)) == np.max(s_j[:, j])

        # Ensures the correct block type is chosen for each task
        for j in range(m):
            self.prob += pulp.lpSum(x[i][j][t][g] * delta[j][t] for i in range(n) for t in range(p) for g in range(q)) == 1

        for t in range(p):
            if not self.block_reusable[t]:
                # Each block is used at most once
                self.prob += pulp.lpSum(x[i][j][t][g] for i in range(n) for j in range(m) for g in range(q)) <= 1
        
        # the grasp side is selected for both pick-and-place and support robot
        for j in range(m):
            for g in range(q):
                self.prob += pulp.lpSum(x[i][j][t][g] for i in range(n) for t in range(p)) * np.max(s_j[:, j]) == pulp.lpSum(y[i][j][g] for i in range(n))

        # Precedence constraints
        for (t1, t2) in precedence_constraints:
            self.prob += pulp.lpSum(x[i][j][t1][g] * (m - j) for i in range(n) for j in range(m) for g in range(q)) >= \
                    pulp.lpSum(x[i][j][t2][g] * (m - j) for i in range(n) for j in range(m) for g in range(q))

        # Workload balancing constraints in a window of size n
        for i in range(n):
            for k in range(m-n+1):
                self.prob += z[i][k] == (pulp.lpSum(x[i][j][t][g] for t in range(p) for j in range(k, k+n) for g in range(q)) \
                                        + pulp.lpSum(y[i][j][g] for j in range(k, k+n) for g in range(q)))

        
        for i in range(n):
            for k in range(m-n+1):
                self.prob += z_min[k] <= z[i][k]
                self.prob += z_max[k] >= z[i][k]

        t_build = time.time()
        print(f"Problem built in {t_build - ts:.2f} seconds")
        # Solve the problem
        self.prob.solve()
        
        t_solve = time.time()
        print(f"Problem solved in {t_solve - t_build:.2f} seconds")
        
        solution = {}
        for v in self.prob.variables():
            solution[v.name] = v.varValue
        

        assignment = []
        for j in range(m):
            robot_idx = -1
            sup_idx = -1
            block_idx = -1
            press_side = -1
            press_offset = -1
            for i in range(n):
                if (robot_idx == -1):
                    for t in range(p):
                        for g in range(q):
                            if (robot_idx == -1) and (solution[f"x_{i}_{j}_{t}_{g}"] == 1):
                                robot_idx = i
                                block_idx = t
                                press_side = int(g // self.num_offsets) + 1
                                press_offset = g % self.num_offsets
            for i in range(n):
                for g in range(q):            
                    if s_j[robot_idx, j] and (sup_idx == -1):
                        if solution[f"y_{i}_{j}_{g}"] == 1:
                            sup_idx = i
            assignment.append((j, robot_idx, block_idx, press_side, press_offset, sup_idx))

        solution['Total Cost'] = pulp.value(self.prob.objective)

        t_parse_sol = time.time()
        print(f"Solution parsed in {t_parse_sol - t_solve:.2f} seconds")
        print(f"Total time: {t_parse_sol - ts:.2f} seconds")

        if (solution['Total Cost'] > self.inf):
            return assignment, 'Infeasible'
        return assignment, pulp.LpStatus[self.prob.status]
    
    def parse_solution(self, config_json, assignment):
        for a in assignment:
            print(f"Task {a[0]} assigned to robot {a[1]+1} and lego block {a[2]} {self.block_names[a[2]]} " + \
                f"press_side {a[3]}, offset {a[4]}, support robot {a[5]+1}")
    
            task_id = f'{a[0]+1}'
            brick_seq = self.block_names[a[2]].split('_')[1]
            if brick_seq.isdigit():
                # the brick seq is a number, meaning it is in a unique location
                config_json[task_id]['brick_seq'] = int(brick_seq)
            else:
                # the brick seq is a string, meaning it is in a station, we add a unique task id to it
                config_json[task_id]['brick_seq'] = f'{brick_seq}.{task_id}'
            config_json[task_id]['press_side'] = a[3]
            config_json[task_id]['press_offset'] = a[4]
            config_json[task_id]['robot_id'] = a[1]+1
            config_json[task_id]['sup_robot_id'] = a[5]+1


def solve_and_write(output_dir: str, task_config_path: str, output_fname: str) -> int:
    solver = TaskAssignmentSolver()
    cost_matrix, delta_matrix, s, s_j, precedence_matrix = solver.load_matrix(output_dir)
    n = cost_matrix.shape[0]
    m = cost_matrix.shape[1]
    p = cost_matrix.shape[2]
    g = cost_matrix.shape[3]
    assignment, status = solver.solve_problem(n, m, p, g, cost_matrix, s, delta_matrix, s_j, precedence_matrix)
    print(f"Status: {status}")

    if status == 'Infeasible':
        print("Task Assignment is infeasible")
        return 1

    with open(task_config_path, 'r') as f:
        task_config = json.load(f)

    solver.parse_solution(task_config, assignment)
    out_path = Path(output_dir) / output_fname
    with open(out_path, 'w') as f:
        json.dump(task_config, f, indent=4)
    print("Task Assignment is feasible")
    print(f"Wrote assignment to {out_path}")
    return 0


def test(task='tower'):
    output_dir = LEGO_CONFIG_DIR / 'steps'
    json_path = LEGO_CONFIG_DIR / 'assembly_tasks' / f'{task}.json'
    return solve_and_write(str(output_dir), str(json_path), "assemb_seq.json")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', type=str, default='tower')
    parser.add_argument('--output-dir', type=str, default='')
    parser.add_argument('--task-config-path', type=str, default='')
    parser.add_argument('--output-fname', type=str, default='')
    args = parser.parse_args()
    if args.output_dir and args.task_config_path and args.output_fname:
        raise SystemExit(solve_and_write(args.output_dir, args.task_config_path, args.output_fname))
    raise SystemExit(test(args.task))
