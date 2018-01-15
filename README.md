# Variational_bayes_gaussian_mixture_models
LightSensorからバグファイルに記録して、変分ベイズによる混合ガウス分布のパラメータ推定を用いたクラスタリングを行うパッケージです。

## Demo
* [VBGMM clustering](https://www.youtube.com/watch?v=rWt3-W2nMFE&feature=youtu.be)

## Requirements
* Raspberry Pi Mouse
* Ubuntu 16.04
* ROS Kinetic
* ROS package
  * [ryuichiueda/raspimouse_ros](https://github.com/ryuichiueda/raspimouse_ros.git)

## Description
センサの値をバグファイルに記録  
バグファイルを読み取り、クラスタリングを行う  
クラスタ数と各クラスタの混合率を表示する


## Installation
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/taishi107/Variational_bayes_gaussian_mixture_models.git
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
$ roscore
$ rosrun raspimouse_ros rtlightsensors.py
$ rosrun Variational_bayes_gaussian_mixture_models logger.py
$ ctrl -C
$ rosrun Variational_bayes_gaussian_mixture_models vbgmm.py
```
## License
このリポジトリは、MITライセンス下です。
