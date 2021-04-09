# 12th DIMACS Implementation Challenge: CVRP track

[![GitHub license](https://img.shields.io/github/license/andremaravilha/DIMACS-Challenge-2021)](https://github.com/andremaravilha/DIMACS-Challenge-2021/blob/main/LICENSE) 
![GitHub last commit](https://img.shields.io/github/last-commit/andremaravilha/DIMACS-Challenge-2021) 
![Lines of code](https://img.shields.io/tokei/lines/github/andremaravilha/DIMACS-Challenge-2021)

> André L. Maravilha<sup>1, 2</sup>  
> <sup>1</sup> *Dept. of Informatics, Management and Design - Centro Fed. de Edu. Tecnológica de Minas Gerais ([url](https://www.cefetmg.br/))*  
> <sup>2</sup> *Operations Research and Complex Systems Lab. - Universidade Federal de Minas Gerais ([url](https://orcslab.github.io/))*

## About

This repository contains the source code and submissions to the CVRP track of the 12th DIMACS Implementation Challenge.

People interested in more details of the challenge can acdess the event's official page through this [url](http://dimacs.rutgers.edu/programs/challenge/vrp/).

## How to build the project

#### Important comments before building the project

To compile this project you need CMake (version 3.13 or later) and a compatible compiler installed on your computer. The code has not been tested on versions earlier than the ones specified.

#### Building the project

Inside the root directory of the project, run the following commands:
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

