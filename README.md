# work on the HEBI robots

- notes.txt explains how to get the software installed and visualize a virtual model of the robot

- networking.txt explains how to connect to the physical robot

- The .py scripts have some code to control the virtual or physical Rosie robot

## Mesh Files

Mesh files are large and not version controlled. You can download them [here](https://sumailsyr-my.sharepoint.com/:u:/g/personal/sparida_syr_edu/IQAjUS5YLESkQoYwHJzJGrM4ASBJQtAQ4VTf42ReL1c72NM?e=m64pd7) (you'll need to be logged into your SU OneDrive account for access). Then extract the zip archive into a sub-folder of this repository at path `./hebi_description/meshes/`

You also need to install the [PyBullet](https://pybullet.org/wordpress/) simulator.

Once PyBullet is installed and the meshes are extracted in the correct location, you can visualize the simulation environment with this command (run from within the root of the repository):
`python rosie_sim.py`
