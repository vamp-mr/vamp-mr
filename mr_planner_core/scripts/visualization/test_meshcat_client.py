#!/usr/bin/env python3
"""
Send a single test scene to the meshcat_bridge JSON socket.
Usage:
  python3 test_meshcat_client.py --host 127.0.0.1 --port 7600
"""
import argparse
import json
import socket
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7600)
    parser.add_argument("--delete-all", action="store_true", help="Send a delete_all first")
    args = parser.parse_args()

    msg = {
        "action": "update",
        "timestamp": time.time(),
        "robot_spheres": [
            {
                "robot": "test_bot",
                "robot_id": 0,
                "link": "link_0",
                "sphere_index": 0,
                "center": [0.2, 0.0, 0.1],
                "radius": 0.05,
                "rgba": [0.1, 0.6, 1.0, 0.8],
            },
            {
                "robot": "test_bot",
                "robot_id": 0,
                "link": "link_1",
                "sphere_index": 1,
                "center": [0.3, 0.1, 0.2],
                "radius": 0.05,
                "rgba": [0.9, 0.4, 0.2, 0.8],
            },
        ],
        "objects": [
            {
                "name": "box_1",
                "type": "box",
                "size": [0.1, 0.05, 0.04],
                "position": [0.0, 0.0, 0.02],
                "quaternion": [0, 0, 0, 1],
                "rgba": [0.2, 0.8, 0.4, 0.9],
            }
        ],
    }

    payload = json.dumps(msg) + "\n"
    with socket.create_connection((args.host, args.port), timeout=2.0) as sock:
        if args.delete_all:
            sock.sendall(json.dumps({"action": "delete_all"}).encode("utf-8") + b"\n")
            print(f"Sent delete_all to {args.host}:{args.port}")
        else:
            sock.sendall(payload.encode("utf-8"))
        print(f"Sent test scene to {args.host}:{args.port}")


if __name__ == "__main__":
    main()
