# murmurations

Murmurations are the name of a flock of startings which exhibit particularly mesmerizing flocking behavior. What is remarkable is the limited communication in the flock. They are not chirping commands to each other or communicating telepathically as far as we known! Their unchoreographed dance emerges as a result of the actions of each individual.

## Algorithms

* 1989 - Craig Reynolds - http://www.red3d.com/cwr/boids/
* 2014 - Daniel J.G. Pearce - http://www.pnas.org/content/111/29/10422.abstract

## Dependencies

* [Eigen3](https://eigen.tuxfamily.org/)
* [GLFW](https://www.glfw.org/)
* [GLAD](https://github.com/Dav1dde/glad)

It should be simple to setup on Linux:

```sh
sudo apt install libeigen3-dev

sudo apt install libglfw3
sudo apt install libglfw3-dev

git clone https://github.com/Dav1dde/glad.git
cd glad
cmake ./
make
sudo cp -a include /usr/local/
sudo cp -a src /usr/local/
cd ..
rm -rf glad
```
