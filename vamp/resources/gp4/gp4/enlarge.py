import trimesh

def scale_mesh(input_path, output_path, scale_factor=1.15):
    mesh = trimesh.load_mesh(input_path)
    mesh.apply_scale(scale_factor)
    mesh.export(output_path)

# List of input and output file paths
input_files = [
    #'link_1.stl',
    #'link_2.stl',
    'link_3.stl',
    'link_4.stl',
    'link_5.stl',
    'link_6.stl'
]

output_files = [
    #'link_1_scaled.stl',
    #'link_2_scaled.stl',
    'link_3_scaled.stl',
    'link_4_scaled.stl',
    'link_5_scaled.stl',
    'link_6_scaled.stl'
]

# Scale each mesh file with the new scale factor
for input_file, output_file in zip(input_files, output_files):
    scale_mesh(input_file, output_file, 1.10)

