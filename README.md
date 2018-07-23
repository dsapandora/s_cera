# Ros Installation
1. Create the package, then install the componente needed for retro gym
2. Create the virtualenv needed.

Ros no soportaba python 3, pero gym tenia que correr usando python 3 asi que hice
la adaptacion de ROS para soportarlo

https://stackoverflow.com/questions/49758578/installation-guide-for-ros-kinetic-with-python-3-5-on-ubuntu-16-04

Estoy usando ros kinect-

me marco error al correr
yaml no lo encuentra
asi que lo voy a instalar pero solo en la carpeta que voy a usar
pip3 install pyyaml
tampoco encontro rospkg
pip3 install rospkg

ImportError: No module named 'catkin_pkg'
pip3 install catkin_pkg

Con esto se pudo ejecutar el server, ahora probemos gym retro.

requerimientos
pip3 install imutils
pip3 install opencv-python
pip3 install scipy
pip3 install pygame


#importar los roms
python3 -m retro.import roms


# Problemas con opencv de ros kinect y el opencv de python3

https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv

Proceder a agregar a mis export darle prioridad a mi entorno de python 3
export PYTHONPATH="./env/lib/python3.5/site-packages:$PYTHONPATH"

Se debe ejecutar

export PYTHONPATH="/home/dsapandora/Documents/citic_workspace/catkin_ws/src/s_cera/env/lib/python3.5/site-packages:$PYTHONPATH"

antes de cada corrida del server
