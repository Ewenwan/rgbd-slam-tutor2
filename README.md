This is the second version of RGBD SLAM tutor. Please visit [my blogs](http://www.cnblogs.com/gaoxiang12) for details: 

---


# 改进的地方大致如下：

    1. 多线程的优化：在建图算法计算时，定位算法没必要等待它结束。它们可以并行运行。
    2. 更好地跟踪：选取参考帧，并对丢失情况进行处理；
    3. 基于外观的回环检测：Appearance based loop closure；
    4. 八叉树建图：Octomap； 八叉树三维点云地图 https://github.com/Ewenwan/octomap
    5. 使用更快的特征：Orb；
    6. 使用TUM数据集，并与标准轨迹进行比较； TUM数据集网址：http://vision.in.tum.de/data/datasets/rgbd-dataset
    7. 在线的Kinect demo；
    8. 代码会写得更像c++风格，而不是像上次的c风格；

# 代码目录
几个文件夹的内容如下：

    bin　　　   　存放编译好的可执行文件；
    src　　　　   存放源代码；
    include　    存放头文件；
    experiment　　存放一些做实验与测试用的源文件；
    config　　    存放配置文件；
    lib　　　　    存放编译好的库文件；
    Thirdparty　　一些小型的依赖库，例如g2o，dbow2，octomap等；

# 我们构建代码的思路是这样的。
    把与slam相关的代码（include和src下）编译成一个库，
    把测试用的程序（experiment下）编译成可执行文件，并链接到这个slam库上。
    举例来说，我们会把orb特征的提取和匹配代码放到库中，
    然后在experiment里写一个程序，
    读一些具体的图片并提取orb特征。
    以后我们也将用这个方式来编写回环检测等模块。
    
    至于为何要放Thirdparty呢？因为像g2o这样的库，版本有时会发生变化。
    所以我们就把它直接放到代码目录里，而不是让读者自己去找g2o的源码，
    这样就可以保证我们的代码在读者的电脑上也能顺利编译。
    但是像 opencv，pcl 这些大型又较稳定的库，我们就交给读者自行编译安装了。
    
    
    
# 安装 Installization

1. Dependencies: OpenCV 2.4.x , [PCL 1.7](http://pointclouds.org/), Eigen3

        OpenCV 2.4.11　　请往opencv.org下载，注意我们没有使用3.1版本，而opencv2系列和3系列在接口上有较大差异。
        如果你用ubuntu，可以通过软件仓库来安装opencv：
        sudo apt-get install libopencv-dev
        
        Eigen3　　　　　　安装 sudo apt-get install libeigen3-dev
         
         
2. Compile third-party libs, including [DBoW2](https://github.com/raulmur/ORB_SLAM2) (for loop closure), a modified version of [g2o](https://github.com/RainerKuemmerle/g2o) (for solving pnp), and the OrbExtractor from [orb-slam2] (https://github.com/raulmur/ORB_SLAM2).

  They are all cmake projects, so just go into the directory and type:
```
    mkdir build
    cmake ..
    make -j2
```
  For g2o you need to type *make install* to install it into /usr/local/ otherwise the FindG2O.cmake will not work.


3. Compile this project:

```
    mkdir build
    cmake ..
    make -j2
```

You will find some experiment binaries in *bin/* like bin/exp_mapping, they are experiments described in the blog. Please download the dataset and edit the parameter file before doing experiments. 

Many thanks to the excellent works of g2o and orb-slam!

# 关于TUM数据集

    本次我们使用tum提供的数据集。tum的数据集带有标准的轨迹和一些比较工具，更适合用来研究。
    同时，相比于nyud数据集，它也要更加困难一些。使用这个数据集时应当注意它的存储格式（当然使用任何数据集都应当注意）。

    下面我们以fr1_room为例来说明TUM数据集的用法。fr1_room的下载方式见上面的百度云或者TUM官网。

    下载我们提供的 “rgbd_dataset_freiburg1_room.tgz”至任意目录，解压后像这样：
    
![](https://images2015.cnblogs.com/blog/606958/201602/606958-20160221132708717-850699130.png)
    
    　rgb和depth文件夹下存放着彩色图和深度图。图像的文件名是以采集时间命名的。
     而rgb.txt和depth.txt则存储了所有图像的采集时间和文件名称，例如：

    　　1305031910.765238 rgb/1305031910.765238.png

    表示在机器时间1305031910.765238采集了一张RGB图像，存放于rgb/1305031910.765238.png中。

    这种存储方式的一个特点是，没有直接的rgb-depth一一对应关系。
    由于采集时间的差异，几乎没有两张图像是同一个时刻采集的。
    然而，我们在处理图像时，需要把一个RGB和一个depth当成一对来处理。
    所以，我们需要一步预处理，找到rgb和depth图像的一一对应关系。

    TUM为我们提供了一个工具来做这件事，
    详细的说明请看：http://vision.in.tum.de/data/datasets/rgbd-dataset/tools 
    该网页整理了一些常用工具，包括时间配对，ground-truth误差比对、图像到点云的转换等。
    对于现在预处理这一步，我们需要的是一个 associate.py 文件，
    如下（你可以直接把内容拷下来，存成本地的associate.py文件）：
    
```python
# -*- utf-8 -*-
#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements: 
# sudo apt-get install python-argparse

"""
The Kinect provides the color and depth images in an un-synchronized way. This means that the set of time stamps from the color images do not intersect with those of the depth images. Therefore, we need some way of associating color images to depth images.

For this purpose, you can use the ''associate.py'' script. It reads the time stamps from the rgb.txt file and the depth.txt file, and joins them by finding the best matches.
"""

import argparse
import sys
import os
import numpy

# 读取=====
def read_file_list(filename):
    """
    Reads a trajectory from a text file. 
    
    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp. 
    
    Input:
    filename -- File name
    
    Output:
    dict -- dictionary of (stamp,data) tuples
    
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)

# 匹配 ========
def associate(first_list, second_list,offset,max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim 
    to find the closest match for every input tuple.
    
    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    
    """
    first_keys = first_list.keys()
    second_keys = second_list.keys()
    potential_matches = [(abs(a - (b + offset)), a, b) 
                         for a in first_keys 
                         for b in second_keys 
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    
    matches.sort()
    return matches

if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script takes two data files with timestamps and associates them   
    ''')
    parser.add_argument('first_file', help='first text file (format: timestamp data)')
    parser.add_argument('second_file', help='second text file (format: timestamp data)')
    parser.add_argument('--first_only', help='only output associated lines from first file', action='store_true')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    args = parser.parse_args()

    first_list = read_file_list(args.first_file)
    second_list = read_file_list(args.second_file)

    matches = associate(first_list, second_list,float(args.offset),float(args.max_difference))    

    if args.first_only:
        for a,b in matches:
            print("%f %s"%(a," ".join(first_list[a])))
    else:
        for a,b in matches:
            print("%f %s %f %s"%(a," ".join(first_list[a]),b-float(args.offset)," ".join(second_list[b])))
            

```

    实际上，只要给它两个文件名即可，它会输出一个匹配好的序列，像这样：
    
    python associate.py rgb.txt depth.txt
    
    输出则是一行一行的数据，如：
    1305031955.536891 rgb/1305031955.536891.png 1305031955.552015 depth/1305031955.552015.png
    这一行就是配对好的RGB图和深度图了
    程序默认时间差在0.02内的就可以当成一对图像。为了保存这个结果，我们可以把它输出到一个文件中去，如：
    python associate.py rgb.txt depth.txt > associate.txt
    这样，只要有了这个associate.txt文件，我们就可以找到一对对的RGB和彩色图啦！
    配对配对什么的，总觉得像在相亲啊……


## 关于ground truth

    ground truth是TUM数据集提供的标准轨迹，
    它是由一个外部的（很高级的）运动捕捉装置测量的，
    基本上你可以把它当成一个标准答案喽！
    ground truth的记录格式也和前面类似，像这样：

    1305031907.2496 -0.0730 -0.4169 1.5916 0.8772 -0.1170 0.0666 -0.4608

    各个数据分别是：时间，位置（x,y,z），姿态四元数（qx, qy, qz, qw），
    对四元数不熟悉的同学可以看看“数学基础”那几篇博客。
    那么这个轨迹长什么样呢？
    我们写个小脚本来画个图看看：
```python

#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d

f = open("./groundtruth.txt")
x = []
y = []
z = []
for line in f:
    if line[0] == '#':
        continue #去掉注释
    data = line.split() # 按空格分开
    x.append( float(data[1] ) )
    y.append( float(data[2] ) )
    z.append( float(data[3] ) )
ax = plt.subplot( 111, projection='3d')
ax.plot(x,y,z) # 显示3维 轨迹点
plt.show()

```
    把这部分代码复制存储成draw_groundtruth.py存放到数据目录中，再运行：
    python draw_groundtruth.py

![](https://images2015.cnblogs.com/blog/606958/201602/606958-20160221135949045-1772096676.png)


    第二件事，因为外部那个运动捕捉装置的记录频率比较高，得到的轨迹点也比图像密集很多，如何查找每个图像的真实位置呢？

    还记得associate.py不？我们可以用同样的方式来匹配associate.txt和groundtruth.txt中的时间信息哦：

    python associate.py associate.txt groundtruth.txt > associate_with_groundtruth.txt

    这时，我们的新文件 associate_with_groundtruth.txt 中就含有每个帧的位姿信息了：

    1305031910.765238 rgb/1305031910.765238.png 1305031910.771502 depth/1305031910.771502.png 
    1305031910.769500 -0.8683 0.6026 1.5627 0.8219 -0.3912 0.1615 -0.3811

    是不是很方便呢？对于TUM中其他的序列也可以同样处理。

关于TUM中的相机

    TUM数据集一共用了三个机器人，记成fr1, fr2, fr3。
    这三台相机的参数在这里： 

    http://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect

    数据当中，深度图已经根据内参向RGB作了调整。所以相机内参以RGB为主：
    Camera          fx      fy      cx    cy    d0      d1      d2      d3      d4
    (ROS default)   525.0  525.0  319.5  239.5 0.0      0.0     0.0     0.0     0.0
    Freiburg 1 RGB  517.3  516.5  318.6  255.3 0.2624  -0.9531  -0.0054 0.0026  1.1633
    Freiburg 2 RGB  520.9  521.0  325.1  249.7 0.23 12 -0.7849  -0.0033 -0.0001  0.9172
    Freiburg 3 RGB  535.4  539.2  320.1  247.6 0          0      0       0      0

    深度相机的scale为5000（和kinect默认的1000是不同的）。也就是depth/中图像像素值5000为真实世界中的一米。
  
# 挑选一个IDE

    现在让我们来写第一部分代码：读取tum数据集并以视频的方式显示出来。

    嗯，在写代码之前呢，师兄还有一些话要啰嗦。虽然我们用linux的同学以会用vim和emacs为傲，但是写代码呢，还是希望有一个IDE可以用的。
    vim和emacs的编辑确实很方便，然而写c++，你还需要在类定义/声明里跳转，需要补全和提示。
    要让vim和emacs来做这种事，不是不可以，但是极其麻烦。
    这次师兄给大家推荐一个可以用于c++和cmake的IDE，叫做qtcreator。

    安装qtcreator：   可能有问题，需要源码安装

    sudo apt-get install qtcreator

# cmakelists.txt
```c
# 主文件 cmakelists.txt=========================
cmake_minimum_required( VERSION 2.8 )# 版本要求
project( rgbd-slam-tutor2 )# 项目名称

# 设置用debug还是release模式。debug允许断点，而release更快
#set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_BUILD_TYPE Release )

# 设置编译选项
# 允许c++11标准、O3优化、多线程。match选项可避免一些cpu上的问题
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread" )

# 常见依赖库：cv, eigen, pcl
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( PCL 1.7 REQUIRED )

include_directories(
${PCL_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/
)# 包含

link_directories(${PCL_LIBRARY_DIRS})# 链接
add_definitions(${PCL_DEFINITIONS})# 定义

set( thirdparty_libs
    ${OpenCV_LIBS}
    ${PCL_LIBRARY_DIRS}
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
)


# 二进制文件输出到bin  可执行文件 EXECUTABLE 
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
# LIBRARY 库输出到lib  
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib )

# 头文件目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    )

# 源文件目录
add_subdirectory( ${PROJECT_SOURCE_DIR}/src/ )
add_subdirectory( ${PROJECT_SOURCE_DIR}/experiment/ )
```

> 次级 cmakelists.txt 文件 src/CMakeLists.txt experiment/CMakeLists.txt
```c
# 直接添加需要编译的文件
# 修改experiment/CMakeLists.txt文件，告诉它我们要编译这个文件：
add_executable( helloslam helloslam.cpp )

```

# 参数文件读取类
    yaml太麻烦了
    方便调试程序，而不用重新编译

> 示例 参数文件 
```
# 这是一个参数文件
# 这虽然只是个参数文件，但是是很厉害的呢！
# 去你妹的yaml! 我再也不用yaml了！简简单单多好！

# 数据相关
# 起始索引
start_index=1
# 数据所在目录
data_source=/home/xiang/Documents/data/rgbd_dataset_freiburg1_room/

# 相机内参

camera.cx=318.6
camera.cy=255.3
camera.fx=517.3
camera.fy=516.5
camera.scale=5000.0
camera.d0=0.2624
camera.d1=-0.9531
camera.d2=-0.0054
camera.d3=0.0026
camera.d4=1.1633

```
    语法很简单，以行为单位，以#开头至末尾的都是注释。
    参数的名称与值用等号相连，即 名称＝值 ，很容易吧！
    下面我们做一个ParameterReader类，来读取这个文件。
> 常用头文件 和结构体 文件
    在此之前，先新建一个 include/common.h 文件，
    把一些常用的头文件和结构体放到此文件中，
    省得以后写代码前面100行都是#include。
```c
#ifndef COMMON_H
#define COMMON_H

/**
 * common.h
 * 定义一些常用的结构体
 * 以及各种可能用到的头文件，放在一起方便include
 */

// C++标准库=========
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
using namespace std;


// Eigen=============
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV=============
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// boost==============
#include <boost/format.hpp>
#include <boost/timer.hpp>
#include <boost/lexical_cast.hpp>// boost 的 lexical_cast 能把字符串转成各种 c++ 内置类型

// pcl ===
// g2o ===

namespace rgbd_tutor  // 命名空间 自己定义===============
{

// 相机内参模型
// 增加了畸变参数，虽然可能不会用到
struct CAMERA_INTRINSIC_PARAMETERS
{
    // 标准内参
    double cx=0, cy=0, fx=0, fy=0, scale=0;
    // 畸变因子
    double d0=0, d1=0, d2=0, d3=0, d4=0;
};



// linux终端的颜色输出
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */


}

#endif // COMMON_H

// 请注意我们使用rgbd_tutor作为命名空间，以后所有类都位于这个空间里。
// 然后，文件里还定义了相机内参的结构，这个结构我们之后会用到，先放在这儿。

```
> 参数读取类 头文件 parameter_reader.h
```c
/*
文件名：parameter_reader.h
功能:  参数读取类
说明:
为保持简单，我把实现也放到了类中。
该类的构造函数里，传入参数文件所在的路径。
在我们的代码里，parameters.txt位于代码根目录下。
不过，如果找不到文件，我们也会在上一级目录中寻找一下，
这是由于qtcreator在运行程序时默认使用程序所在的目录（./bin）而造成的。

　　ParameterReader 实际存储的数据都是std::string类型（字符串），
    在需要转换为其他类型时，我们用 boost::lexical_cast 进行转换。

　　ParameterReader::getData 函数返回一个参数的值。
 
   它有一个模板参数，你可以这样使用它：
　　double d = parameterReader.getData<double>("d");
　　如果找不到参数，则返回一个空值。
　　最后，我们还用了一个函数返回相机的内参，这纯粹是为了外部类调用更方便。
  
*/
#ifndef PARAMETER_READER_H
#define PARAMETER_READER_H

#include "common.h"

namespace rgbd_tutor
{

class ParameterReader
{
public:
    // 构造函数：传入参数文件的路径
    ParameterReader( const string& filename = "./parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            // 看看上级目录是否有这个文件 ../parameter.txt
            fin.open("."+filename);
            if (!fin)
            {
                cerr<<"没有找到对应的参数文件："<<filename<<endl;
                return;
            }
        }

        // 从参数文件中读取信息
        while(!fin.eof()) // 直到末尾
        {
            string str;
            getline( fin, str );
            if (str[0] == '#') // 跳过 # 开头的
            {
                // 以‘＃’开头的是注释
                continue;
            }
            
            int pos = str.find('#'); // 其余的行中 #到末尾都是注释========
            if (pos != -1)
            {
                // 从井号到末尾的都是注释
                str = str.substr(0, pos); // 0开始到#号前才是 参数内容====
            }

            // 查找等号==========================
            pos = str.find("=");
            if (pos == -1)
                continue;//未找打=
            // 等号左边是key，右边是value===========
            string key = str.substr( 0, pos );// 左边
            string value = str.substr( pos+1, str.length() );// 右边
            data[key] = value; // 字典类

            if ( !fin.good() )
                break;
        }
    }

    // 获取数据
    // 由于数据类型不确定，写成模板===========================
    template< class T >
    T getData( const string& key ) const
    {
        auto iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return boost::lexical_cast<T>( "" );
        }
        
        // boost 的 lexical_cast 能把字符串转成各种 c++ 内置类型==========
        return boost::lexical_cast<T>( iter->second );
    }

    // 直接返回读取到的相机内参===========================================
    rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS getCamera() const
    {
        static rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS camera;
        camera.fx = this->getData<double>("camera.fx");
        camera.fy = this->getData<double>("camera.fy");
        camera.cx = this->getData<double>("camera.cx");
        camera.cy = this->getData<double>("camera.cy");
        camera.d0 = this->getData<double>("camera.d0");
        camera.d1 = this->getData<double>("camera.d1");
        camera.d2 = this->getData<double>("camera.d2");
        camera.d3 = this->getData<double>("camera.d3");
        camera.d4 = this->getData<double>("camera.d4");
        camera.scale = this->getData<double>("camera.scale");
        return camera;
    }

protected:
    map<string, string> data;
};

};

#endif // PARAMETER_READER_H

```


# RGBD相机帧类  include/rgbdframe.h  数据类 读取类(数据集)
    程序运行的基本单位是Frame，而我们从数据集中读取的数据也是以Frame为单位的。
    现在我们来设计一个RGBDFrame类，以及向数据集读取Frame的FrameReader类。
    
> feature.h
   
```c
// 每个特征点
#ifndef FEATURE_H
#define FEATURE_H

#include "common_headers.h"

namespace rgbd_tutor
{

class Feature
{
public:
    Feature( ) {}
public:
    cv::KeyPoint    keypoint;    // 关键点
    cv::Mat         descriptor;  // 描述子
    cv::Point3f     position;    // 2d 关键点对应的 3d点position in 3D space
    float           observe_frequency = 0.0;  // 被观测到的频率 被相机帧观测到的次数
};

}

#endif // FEATURE_H

```


> rgbdframe.h

```c
#ifndef RGBDFRAME_H
#define RGBDFRAME_H

#include "common.h"
#include "parameter_reader.h" // 参数读取==================

#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

#include "feature.h"   // 特征结构
#include "utils.h"     // 一些转换的函数

// pcl
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

// 多线程 互斥量=====
#include <mutex> 

/**
 * RGBDFrame 
 * 该类记录了每个帧的信息。帧是slam的基本单位。
 * 本身可以看作一个struct。
 * 由于Pose可能被若干线程同时访问，需要加锁。
 */

namespace rgbd_tutor{

//帧类===============数据结构=============================================
class RGBDFrame//帧
{
public:
    typedef shared_ptr<RGBDFrame> Ptr;// 类 对象 共享指针，不用考虑 delet
// 我们把这个类的指针定义成了shared_ptr，以后尽量使用这个指针管理此类的对象，这样可以免出一些变量作用域的问题。
// 并且，智能指针可以自己去delete，不容易出现问题。

public:
    // 数据成员============================================
    int id  =-1;            //-1表示该帧不存在
    
    std::mutex mutexT;// 由于Pose可能被若干线程同时访问，需要加锁。
 
    // 彩色图和深度图
    cv::Mat rgb, depth;
    // 该帧位姿
    // 定义方式为：x_local = T * x_world 注意也可以反着定义； 从世界坐标系 到 当前帧坐标系======
    Eigen::Isometry3d       T=Eigen::Isometry3d::Identity();// 初始化为 单位矩阵

    // 特征
    
    vector<cv::KeyPoint>    keypoints; // 关键点
    cv::Mat                 descriptor;// 描述子
    vector<cv::Point3f>     kps_3d;    // 2d 关键点对应的 3d点
// 后改为一个  
// 特征
// vector<Feature> features;


    // 相机
    // 默认所有的帧都用一个相机模型（难道你还要用多个吗？）
    CAMERA_INTRINSIC_PARAMETERS camera;

    // BoW回环特征
    // 讲BoW时会用到，这里先请忽略之
    DBoW2::BowVector bowVec;// 特征 词带=====================



public:
    RGBDFrame() {} // 空的构造函数=============================
    // 方法===================================================
    // 给定像素点，使用深度数据，获取 本帧坐标系下的3D点坐标========
    // 可以把一个像素坐标转换为当前Frame下的3D坐标。当然前提是深度图里探测到了深度点。
    cv::Point3f project2dTo3dLocal( const int& u, const int& v  ) const
    {
        if (depth.data == nullptr)
            return cv::Point3f();
        ushort d = depth.ptr<ushort>(v)[u];// 获取v 行 u列 的向上对应的深度值================
        if (d == 0)
            return cv::Point3f();
            
        cv::Point3f p;                    // 三角测量计算3d点
        p.z = double( d ) / camera.scale;
        p.x = ( u - camera.cx) * p.z / camera.fx;
        p.y = ( v - camera.cy) * p.z / camera.fy;
        return p;
    }

    // 将一组descriptor转换为一个矩阵 Mat ===================
    cv::Mat getAllDescriptors ( ) const
    {
        cv::Mat desp;
        for ( size_t i=0; i<features.size(); i++ )
        {
            desp.push_back( features[i].descriptor );
        }
        return desp;
    }

    // 获取所有的descriptor并组成一个 向量 vector===========
    vector<cv::Mat> getAllDescriptorsVec() const
    {
        vector<cv::Mat> desp;
        for ( auto f:features )
        {
            desp.push_back( f.descriptor );
        }
        return desp;
    }

    // 获取所有的keypoints===========向量vector==================
    vector<cv::KeyPoint>    getAllKeypoints() const
    {
        vector<cv::KeyPoint> kps;
        for ( auto f:features )
        {
            kps.push_back( f.keypoint );
        }
        return kps;
    }
    
    // 有关于 帧位姿 数据 的读写 要加锁========================
    void setTransform( const Eigen::Isometry3d& T )
    {
        std::unique_lock<std::mutex> lck(mutexT);
        T_f_w = T; // 写 设置 帧位姿态
    }
    
    Eigen::Isometry3d getTransform()  
    {
        std::unique_lock<std::mutex> lck(mutexT);
        return T_f_w;// 读  获取 帧位姿态
    }

};

// FrameReader 帧数据读取类 ======================================
// 从TUM数据集中读取数据的类   主要用于数据集====================
class FrameReader
{
public:
    FrameReader( const rgbd_tutor::ParameterReader& para )
        : parameterReader( para )
    {
        init_tum( );
    }

    // 获得下一帧
    RGBDFrame::Ptr   next();

    // 重置index
    void    reset()
    {
        cout<<"重置 frame reader"<<endl;
        currentIndex = start_index;
    }

    // 根据index获得帧
    RGBDFrame::Ptr   get( const int& index )
    {
        if (index < 0 || index >= rgbFiles.size() )
            return nullptr;
        currentIndex = index;
        return next();
    }

protected:
    // 初始化tum数据集
    void    init_tum( );
protected:

    DATASET dataset_type =TUM; // 数据类型
    
    // 当前索引
    int currentIndex =0;
    // 起始索引
    int start_index  =0;

    const   ParameterReader&    parameterReader;

    // 文件名序列
    vector<string>  rgbFiles, depthFiles;

    // 数据源
    string  dataset_dir;

    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS     camera;
};

};
#endif // RGBDFRAME_H

include/rgbdframe.h

```
> 我们在src/rgbdframe.cpp中实现init_tum()和next()这两个函数 src/rgbdframe.cpp

```c
#include "rgbdframe.h"
#include "common.h"
#include "parameter_reader.h"

using namespace rgbd_tutor; // 命名空间=========================================

RGBDFrame::Ptr   FrameReader::next()
{
    if (currentIndex < start_index || currentIndex >= rgbFiles.size())
        return nullptr; // 没有这一帧

    RGBDFrame::Ptr   frame (new RGBDFrame);
    frame->id = currentIndex;
    frame->rgb = cv::imread( dataset_dir + rgbFiles[currentIndex]); // rgb图============
    frame->depth = cv::imread( dataset_dir + depthFiles[currentIndex], -1);// 彩色图=====

    if (frame->rgb.data == nullptr || frame->depth.data==nullptr)
    {
        // 数据不存在
        return nullptr;
    }

    frame->camera = this->camera;
    currentIndex ++;
    return frame;
}

void FrameReader::init_tum( )
{
    dataset_dir = parameterReader.getData<string>("data_source"); // 数据源
    string  associate_file  =   dataset_dir+"/associate.txt";     // rgb depth 匹配数据
    ifstream    fin(associate_file.c_str());
    if (!fin)
    {
        cerr<<"找不着assciate.txt啊！在tum数据集中这尼玛是必须的啊!"<<endl;
        cerr<<"请用python assicate.py rgb.txt depth.txt > associate.txt生成一个associate文件，再来跑这个程序！"<<endl;
        return;
    }

    while( !fin.eof() )
    {
        string rgbTime, rgbFile, depthTime, depthFile;
        fin>>rgbTime>>rgbFile>>depthTime>>depthFile;
        if ( !fin.good() )
        {
            break;
        }
        rgbFiles.push_back( rgbFile ); // rgb 图像文件
        depthFiles.push_back( depthFile );// depth文件
    }

    cout<<"一共找着了"<<rgbFiles.size()<<"个数据记录哦！"<<endl;
    camera = parameterReader.getCamera(); // 相机参数
    start_index = parameterReader.getData<int>("start_index");//起始帧id 
    currentIndex = start_index;
}


```

> 测试FrameReader
    
    现在我们来测试一下之前写的FrameReader。在experiment中添加一个reading_frame.cpp文件，测试文件是否正确读取。
    experiment/reading_frame.cpp
```c

#include "rgbdframe.h"

using namespace rgbd_tutor; // 命名空间
int main()
{
    ParameterReader para;
    FrameReader     fr(para);//参数读取类初始化 帧读取类 ==============
    while( RGBDFrame::Ptr frame = fr.next() ) // 读取
    {
        cv::imshow( "image", frame->rgb ); // 显示rgb
        cv::waitKey(1);
    }

    return 0;
}

```


> 编译

    我们把rgbdframe.cpp编译成了库，然后把reading_frame链接到了这个库上。
    由于在RGBDFrame类中用到了DBoW库的代码，所以我们先去编译一下DBoW这个库。
    
    cd Thirdparty/DBoW2
    mkdir build lib
    cd build
    cmake ..
    make -j4
    这样就把DBoW编译好了。这个库以后我们要在回环检测中用到。
    
src/目录下的CMakeLists.txt：

    add_library( rgbd_tutor  # 编译成依赖库
         rgbdframe.cpp
     )
experiment下的CMakeLists.txt

    add_executable( helloslam helloslam.cpp )
    
    # 测试 帧读取类=======================
    add_executable( reading_frame reading_frame.cpp )
    target_link_libraries( reading_frame rgbd_tutor ${thirdparty_libs} )  # 需要依赖 src生成的库
    
    编译后在bin/下面生成reading_frame程序，可以直接运行。
    随后我们要把特征提取、匹配和跟踪都加进去，但是希望它仍能保持在正常的视频速度。




# orb 特征提取类  基于 orbslam2的特征提取部分

```
//  > /include/orb.h
#ifndef ORB_H
#define ORB_H

#include "common_headers.h"
#include "rgbdframe.h"
#include "Thirdparty/orbslam_modified/include/ORBextractor.h" // orbslam2的特征提取部分====

/**
 * OrbFeature
 * 对orbslam2的特征部分进行一次封装，具有提取、匹配特征的功能
 */

namespace rgbd_tutor
{

class OrbFeature
{
public:
    OrbFeature( const rgbd_tutor::ParameterReader& para ) // 传入参数读取对象
    {
        int features  = para.getData<int>("orb_features"); // 特征类型
        float   scale = para.getData<float>("orb_scale");  // 金字塔尺度
        int     level = para.getData<int>("orb_levels");   // 金字塔层数
        int     ini   = para.getData<int>("orb_iniThFAST");// ORB FAST焦点阈值
        int     min   = para.getData<int>("orb_minThFAST");
        
        // 使用了 ORB_SLAM2的 特征ORB提取
        extractor = make_shared<ORB_SLAM2::ORBextractor>( features, scale, level, ini, min );// 共享提取器====
        matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");// 暴力匹配==蛮力匹配===
        knn_match_ratio = para.getData<double>("knn_match_ratio");    // 最匹配 < 次匹配 比值!!!!!!
    }

    // 提取特征，存放到frame的成员变量中
    void detectFeatures( rgbd_tutor::RGBDFrame::Ptr& frame ) const
    {
        cv::Mat gray = frame->rgb; // 彩色图=================

        if (frame->rgb.channels() == 3)
        {
            // The BGR image
            cv::cvtColor( frame->rgb, gray, cv::COLOR_BGR2GRAY );// 转换成灰度图======
        }

        vector<cv::KeyPoint>    kps; // 关键点
        cv::Mat     desps;           // 描述子
        (*extractor) ( gray, cv::Mat(), kps, desps);// 提取关键点&描述子
        for ( size_t i=0; i<kps.size(); i++ )
        {
            rgbd_tutor::Feature feature;
            feature.keypoint = kps[i]; // 关键点
            feature.descriptor  = desps.row(i).clone();// 描述子
            feature.position = frame->project2dTo3d( kps[i].pt.x, kps[i].pt.y );// 3d点
            frame->features.push_back( feature );// 特征对象
        }
    }

    // 匹配两个帧之间的特征描述
    vector<cv::DMatch>  match( const rgbd_tutor::RGBDFrame::Ptr& frame1, 
                               const rgbd_tutor::RGBDFrame::Ptr& frame2 ) const;

protected:
    shared_ptr<ORB_SLAM2::ORBextractor> extractor; // 特征和描述子提取器
    cv::Ptr< cv::DescriptorMatcher > matcher;      // 匹配器======

    double knn_match_ratio =0.8;

};

}

#endif // ORB_H

```

> orb.cpp  特征匹配函数的实现========
```c

/*************************************************************************
	> File Name: orb.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年02月29日 星期一 12时14分06秒
 ************************************************************************/

#include <iostream>
#include "common_headers.h"
#include "converter.h" // 转换
#include "orb.h"
using namespace std;

using namespace rgbd_tutor;

vector<cv::DMatch> OrbFeature::match( const RGBDFrame::Ptr& frame1, 
                                      const RGBDFrame::Ptr& frame2 ) const
{
    vector< vector<cv::DMatch> > matches_knn;   // 2维 匹配 结果
    cv::Mat desp1 = frame1->getAllDescriptors(); // 描述子
    cv::Mat desp2 = frame2->getAllDescriptors();
    matcher->knnMatch( desp1, desp2, matches_knn, 2 );// knn 搜索蛮力匹配 获取两个最相似的 匹配
    vector< cv::DMatch > matches;
    for ( size_t i=0; i<matches_knn.size(); i++ )
    {
        if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance ) 最匹配 < 次匹配 比值!!!!!!
        matches.push_back( matches_knn[i][0] ); // 是可靠的 匹配=======
    }
    return matches;
}


```


#  数据匹配 2d-3d pnp 求解类  直接 G2O优化求解 
> include/pnp.h

```c
#ifndef PNP_H
#define PNP_H

#include "common_headers.h"
#include "rgbdframe.h"
#include "parameter_reader.h"
#include "orb.h"

/*
 * 求解pnp问题 获取初始解========
 * 使用g2o作为优化方法求解
 */

namespace rgbd_tutor
{
struct PNP_INFORMATION
{
    // 记录pnp过程中的信息
    int numFeatureMatches   =0;         // 匹配到的特征 数量
    int numInliers          =0;         // pnp内点     数量
    Eigen::Isometry3d   T   = Eigen::Isometry3d::Identity(); //相对变换

};

class PnPSolver
{
public:
    PnPSolver( const rgbd_tutor::ParameterReader& para, 
               const rgbd_tutor::OrbFeature& orbFeature ):
        parameterReader( para ),
        orb( orbFeature )
    {
        min_inliers = para.getData<int>("pnp_min_inliers"); // pnp ransac 最小内点数量
        min_match = para.getData<int>("pnp_min_matches");   // 特征点匹配 最小数量
    }

    //  求解pnp问题
    //  输入  2d点-3d点，相机内参
    //  输出  变换矩阵T（可设置初值），inliers index
    //  返回  是否成功
    bool    solvePnP( const vector<cv::Point2f>& img, 
                      const vector<cv::Point3f>& obj,
                      const rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS& camera, vector<int>& inliersIndex,
                      Eigen::Isometry3d& transform );

    //  更懒的求解方式：直接给定两个帧，匹配由内部计算
    //  输入  两个帧
    //  输出  变换矩阵和inlier index
    bool    solvePnPLazy( const rgbd_tutor::RGBDFrame::Ptr & frame1, 
                          const rgbd_tutor::RGBDFrame::Ptr frame2, 
                          PNP_INFORMATION& pnp_information,
                          bool drawMatches=false );

protected:
    const   rgbd_tutor::ParameterReader& parameterReader;
    const   rgbd_tutor::OrbFeature& orb;

    // 参数
    int     min_inliers =10;
    int     min_match   =30;
};

}

#endif // PNP_H


```


>  	pnp.cpp  数据匹配 2d-3d pnp 求解类

```c
#include "pnp.h"
#include "converter.h"
#include "orb.h"

//  求解pnp问题
//  输入  2d点-3d点，相机内参
//  输出  变换矩阵T（可设置初值），inliers index
//  返回  是否成功
bool rgbd_tutor::PnPSolver::solvePnP( const vector<cv::Point2f>& img, 
                                      const vector<cv::Point3f>& obj,
                                      const rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS& camera, 
                                      vector<int>& inliersIndex, Eigen::Isometry3d& transform )
{
    // g2o初始化===============================================
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = 
                         new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType> ();
    g2o::BlockSolver_6_3*   solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    optimizer.setAlgorithm( solver );

    // 加入待估计的位姿：一个se3 vertex
    g2o::VertexSE3Expmap*   vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate( rgbd_tutor::Converter::toSE3Quat( Eigen::Isometry3d::Identity()) );
    vSE3->setFixed( false );
    // id为零
    vSE3->setId(0);
    optimizer.addVertex( vSE3 );

    // 接下来就是一堆边，边类型为se3 project xyz only pose
    // 这种边只有一个端点，就是se3本身
    // 先用一个vector装起来，之后要用
    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> edges;

    // 谜之delta =======================================
    const float delta = sqrt(5.991);
    // 每条边是否为inliers
    vector<bool>    inliers( img.size(), true );
    int good = 0;
    for ( size_t i=0; i<obj.size(); i++ )
    {
        if (obj[i] == cv::Point3f(0,0,0))
        {
            // 该点值不存在
            inliers[i] = false;
            continue;
        }
        good++;

        g2o::EdgeSE3ProjectXYZOnlyPose * edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        // 2D点通过setmeasurement来设
        edge->setMeasurement( Eigen::Vector2d(img[i].x, img[i].y) );
        // 这种edge比较特殊，3D点和相机参数直接为成员变量
        edge->fx = camera.fx;
        edge->fy = camera.fy;
        edge->cx = camera.cx;
        edge->cy = camera.cy;
        edge->Xw = Eigen::Vector3d( obj[i].x, obj[i].y, obj[i].z );
        // information其实没多大意义，但为了能求解还是要设一个
        edge->setInformation( Eigen::Matrix2d::Identity()*1 );
        // 由于误匹配的存在，要设置robust kernel
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
        edge->setRobustKernel( rk );
        rk->setDelta( delta );
        optimizer.addEdge( edge );
        edge->setId( i );
        edges.push_back(edge);
    }

    // 使用g2o来判断inliers
    // 一共进行四轮迭代，每轮迭代十次
    for(size_t it=0; it<4; it++)
    {
        vSE3->setEstimate( rgbd_tutor::Converter::toSE3Quat( transform ) );
        optimizer.initializeOptimization(0);
        optimizer.optimize( 10 );

        for ( size_t i=0; i<edges.size(); i++ )
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = edges[i];
            if ( inliers[ e->id() ] == true )
            {
                e->computeError();
            }
            //如果某条边的均方误差太大，说明它是一条outlier
            if ( e->chi2() > 5.991 )
            {
                inliers[ e->id() ] = false;
                e->setLevel(1);
                good -- ;
            }
            else
            {
                // 否则就是inlier
                inliers[i] = true;
                e->setLevel(0);
            }

            // 去掉较大误差的边后，就不必再用robust kernel了
            if (it==2)
                e->setRobustKernel( nullptr );
        }

        // 如果inlier太少，就中断
        if (good < 5)
            break;
    }

    for ( size_t i=0; i<inliers.size(); i++ )
    {
        if ( inliers[i] )
        {
            inliersIndex.push_back(i);
        }
    }

    g2o::VertexSE3Expmap* vSE_recov = dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0));
    g2o::SE3Quat    se3_recov = vSE_recov->estimate();

    transform = Eigen::Isometry3d( se3_recov );

    if (inliers.size() > min_inliers)
        return true;
    return false;
}

bool rgbd_tutor::PnPSolver::solvePnPLazy( const rgbd_tutor::RGBDFrame::Ptr & frame1, 
                                          const rgbd_tutor::RGBDFrame::Ptr frame2, 
                                          PNP_INFORMATION& pnp_information, bool drawMatches )
{
    vector<cv::DMatch>  matches = orb.match( frame1, frame2 );
    if ( matches.size() <= min_match )
        return false;
    vector<cv::Point3f> obj;
    vector<cv::Point2f> img;

    vector<cv::DMatch>  validMatches;

    for (auto m:matches)
    {
        cv::Point3f pObj = frame1->features[m.queryIdx].position;
        if (pObj == cv::Point3f(0,0,0))
            continue;
        if (drawMatches)
        {
            validMatches.push_back(m);
        }

        obj.push_back( pObj );
        img.push_back( frame2->features[m.trainIdx].keypoint.pt );
    }

    if ( img.size() <= min_match || obj.size() <= min_match )
        return false;

    vector<int> inliersIndex;
    Eigen::Isometry3d   init_transform = frame1->T_f_w.inverse() * frame2->T_f_w;
    //cout<<"init transform = "<<init_transform.matrix()<<endl;
    bool b = solvePnP( img, obj, frame1->camera, inliersIndex, init_transform );

    pnp_information.numFeatureMatches = img.size();
    pnp_information.numInliers = inliersIndex.size();
    pnp_information.T = init_transform;

    if (drawMatches == true && b==true)
    {
        vector<cv::DMatch> inlierMatches;
        for ( int index:inliersIndex )
            inlierMatches.push_back( validMatches[index] );
        cv::Mat out;
        cv::drawMatches(frame1->rgb, frame1->getAllKeypoints(),
                        frame2->rgb, frame2->getAllKeypoints(),
                        inlierMatches, out );
        cv::imshow( "inlier matches", out );
    }

    if ( pnp_information.numInliers < min_inliers )
    {
        return false;
    }
    return true;

}


```

> pnp 求解类  测试代码

```c
#include "pnp.h"
#include "common_headers.h"

using namespace std;
using namespace rgbd_tutor;

int main()
{
    cout<<"running test pnp"<<endl;
    ParameterReader para;
    FrameReader frameReader( para );
    OrbFeature  orb(para);
    PnPSolver   pnp(para, orb);

    RGBDFrame::Ptr refFrame = frameReader.next();
    orb.detectFeatures( refFrame );// 特征点检测
    Eigen::Isometry3d   speed = Eigen::Isometry3d::Identity(); // 开始的速度

    while (1)
    {
        cout<<"*************************************"<<endl;
        boost::timer timer;
        RGBDFrame::Ptr currFrame = frameReader.next();// 当前帧====

        if ( currFrame == nullptr )
        {
            break;
        }
        currFrame->T_f_w = speed * refFrame->T_f_w ;// 安照横速模型设置 当前帧 位姿 的初始值
        orb.detectFeatures( currFrame );

        PNP_INFORMATION info;
        bool result = pnp.solvePnPLazy( refFrame, currFrame, info, true );// 和参考帧 pnp 2d-3d G2O求解

        if ( result == false )
        {
            cout<<"pnp failed"<<endl;
            refFrame = currFrame;
            cv::waitKey(0);
        }
        else
        {
            currFrame->T_f_w = info.T * refFrame->T_f_w; // 利用优化结果更新 当前帧
            cout<<"result.T="<<endl;
            cout<<info.T.matrix()<<endl;
            cout<<"current = "<< endl << currFrame->T_f_w.matrix() << endl;
            speed = info.T;
            refFrame = currFrame; // 当前帧 更新为 参考帧==========
            cv::waitKey(1);
        }

        cout<<GREEN<<"time used = "<<timer.elapsed()<<RESET<<endl;

    }

    return 0;
}


```


# 跟踪线程 ======= track 
```c
// include/track.h

#ifndef TRACK_H
#define TRACK_H

/**
  * the tracker
  * Tracker跟踪当前输入的帧, 在丢失的时候进行重定位
  * 单纯使用tracker时可能会漂移，它需要后端pose graph提供一个重定位
  * 
  * 算法描述
  * Tracker 内部有 初始化、跟踪成功 和 丢失 三个态
  * 每次用 updateFrame放入一个新帧，记为F
  * Tracker 把新帧F 与它自己维护的一个 候选队列 里的 参考帧相匹配。建立Local bundle adjustment.
  * Local BA 是一个简易的pose graph.
  * 匹配成功时，把当前帧作为新的候选帧，放到候选队列末尾。如果队列太长则删除多余的帧。
  * 队列中的候选帧有一定间隔。
  * 如果整个候选队列都没匹配上，认为此帧丢失。开始丢失计数。
  * 丢失计数大于一定值时进入丢失态。这样做是为了防止因抖动等原因产生的模糊。
  * 如果进入丢失态，可能有两种情况：暂时性遮挡或 kidnapped 。
  * 对于遮挡，最好的策略是原地等待； 对于kidnapped，则应该重置并通知pose graph，使用回环检测来确定全局位置。
  *  
  */

#include "common_headers.h"
#include "parameter_reader.h"
#include "rgbdframe.h"
#include "orb.h"
#include "pnp.h"

#include <thread>   // 线程
#include <mutex>    // 锁
#include <functional>// 用于表示函数对象(function object)的类模板

using namespace rgbd_tutor;

namespace rgbd_tutor 
{

class PoseGraph; // 使用位姿图 进行优化

class Tracker
{
    
public:
    typedef shared_ptr<Tracker> Ptr; // 跟踪类 共享智能指针
    enum    trackerState // 跟踪状态
    {
        NOT_READY=0, // 未准备好
        OK,          // 跟踪正常
        LOST         // 跟踪丢失
    };

public:
    //  公共接口
    Tracker( const rgbd_tutor::ParameterReader para ) :
        parameterReader( para )
    {
        orb = make_shared<rgbd_tutor::OrbFeature> (para);   // 特征点 提取 匹配
        pnp = make_shared<rgbd_tutor::PnPSolver> (para, *orb);// 位姿求解
        max_lost_frame = para.getData<int>("tracker_max_lost_frame");// 丢失情况判断阈值
        refFramesSize = para.getData<int>("tracker_ref_frames");     // 参考帧队列大小
    }

    void setPoseGraph( shared_ptr<PoseGraph> poseGraph_ )
    {
        this->poseGraph = poseGraph_; // 设置位姿图优化 对象指针============
    }
    //  放入一个新帧，返回此帧所在的姿态
    Eigen::Isometry3d    updateFrame( RGBDFrame::Ptr& newFrame );

    trackerState getState() const { return state; } // 返回跟踪器状态=======

    // adjust the frame according to the given ref frame
    // 应该是用于 本帧丢失时，尝试对本帧进行重定位===============================
    // 根据所给参考帧对本帧进行位姿跟踪，设置位姿================================
    bool    adjust( const RGBDFrame::Ptr& ref )
    {
        unique_lock<mutex> lck(adjustMutex);// 其他线程 会读取，需要改变 数据，对数据进行上锁========
        cout<<"adjust frame frame "<<ref->id<<" to "<<currentFrame->id<<endl;
        PNP_INFORMATION info;
        if (pnp->solvePnPLazy( ref, currentFrame, info) == true)// 和参考帧进行匹配 g2o优化求解位
        {
            currentFrame->setTransform( info.T*ref->getTransform() );// 设置全局位姿
            refFrames.clear();// 队列清空
            refFrames.push_back(ref);// 加入参考帧
            cntLost = 0; // 丢失计数清零
            state = OK;  // 跟踪状态 ok
            cout<<"adjust ok"<<endl;
            return true;
        }
        cout<<"adjust failed."<<endl;
        return false;
    }

protected:

    // 私有方法
    // 第一帧时初始化，对新帧提取特征并作为refframe
    void    initFirstFrame( );
    // 正常的track，比较current和refframe
    void    trackRefFrame();
    // 丢失恢复
    void    lostRecover();

protected:
    // 数据

    // 当前帧
    rgbd_tutor::RGBDFrame::Ptr  currentFrame    =nullptr;// 当前帧指针

    // 当前帧的参考帧
    // 队列结构，长度固定
    deque< RGBDFrame::Ptr >  refFrames; // 参考帧 指针 队列 
    int refFramesSize   =5;
    Eigen::Isometry3d   lastPose = Eigen::Isometry3d::Identity();
    
    // 速度
    Eigen::Isometry3d   speed;

    // 参数
    const rgbd_tutor::ParameterReader&   parameterReader;

    // 状态
    trackerState    state   =NOT_READY;

    // 参数
    int     cntLost =0;
    int     max_lost_frame  =5;

    // pose graph 
    shared_ptr<PoseGraph>   poseGraph =nullptr;// 位姿图优化 对象指针
    mutex   adjustMutex; // 调整 锁

protected:
    // 其他用途
    shared_ptr<rgbd_tutor::PnPSolver>   pnp;// 位姿求解
    shared_ptr<rgbd_tutor::OrbFeature>  orb;// 特征点 提取 匹配 

};

}
#endif // TRACK_H

```

> 跟踪类 实现
```c
#include "track.h"
#include <unistd.h>

// tracker的主线程
using namespace rgbd_tutor;

// 处理新的一帧============================================================
Eigen::Isometry3d    Tracker::updateFrame( RGBDFrame::Ptr& newFrame )
{
    unique_lock<mutex> lck(adjustMutex); // 上锁======
    currentFrame = newFrame;// 更新为当前帧=====
    if ( state == NOT_READY ) // 1. 进行初始化
    {
        initFirstFrame( );// 初始化第一帧====成功会改变跟踪状态=========
        return Eigen::Isometry3d::Identity(); // 初始帧作为世界坐标系
    }
    if ( state == OK )// 2. 跟踪正常
    {
        trackRefFrame();// 跟踪参考帧===设置当前帧的位姿=====
        return currentFrame->getTransform();// 返回当前帧的位姿=====
    }
    
    // state = LOST  3. 位置丢失
    lostRecover();// 进行重定位==============
    return currentFrame->getTransform();

}

// 初始化第一帧=============
void    Tracker::initFirstFrame( )
{
    orb->detectFeatures( currentFrame ); // 特征点检测
    refFrames.push_back(currentFrame);   // 加入参考帧队列======
    speed = Eigen::Isometry3d::Identity();// 设置速度
    state = OK;                           // 切换 跟踪器状态
}

void    Tracker::trackRefFrame()
{
    //adjustMutex.lock();
    
    // 初始值 ===== 这里加速速度恒定，使用 速度(位姿差) *上一参考帧  当前帧位姿初始值====
    currentFrame->setTransform( speed * refFrames.back()->getTransform() );
    orb->detectFeatures( currentFrame );// 检测当前帧 特征点
    
    // 使用 参考帧队列中的帧 进行 局部地图跟踪===============================
    // build local BA
    vector<cv::Point3f> obj;// 3d点
    vector<cv::Point2f> img;// 2d点
    for (auto pFrame: refFrames)// 参考帧队列中的每一帧 
    {
        vector<cv::DMatch> matches = orb->match( pFrame, currentFrame );// 当前帧和参考帧进行特征点描述子匹配
        vector<cv::DMatch>  validMatches;
        Eigen::Isometry3d invPose = pFrame->getTransform().inverse();
	// 参考帧坐标逆变换，用于将帧坐标系下的3d点变换到世界坐标下============!!!!!!!!!!!!!!!!!!
	
        for (auto m:matches)
        {
            cv::Point3f pObj = pFrame->features[m.queryIdx].position;// 参考帧对应 坐标系下的3d点
            if (pObj == cv::Point3f(0,0,0))
                continue;// 该3d点不存在
            Eigen::Vector4d vec = invPose * Eigen::Vector4d(pObj.x, pObj.y, pObj.z,1 );// 逆变换到世界坐标系下========
            obj.push_back( cv::Point3f(vec(0), vec(1), vec(2) ) ); // 参考帧世界坐标系 3d点
            img.push_back( currentFrame->features[m.trainIdx].keypoint.pt );// 当前帧 2d像素点
        }
    }
    
    if ( img.size() < 15 )// 总的匹配点数量过少，有可能丢失了
    {
        cntLost ++;// 丢失计数
        if (cntLost > max_lost_frame)
        {// 如果 大于 最大丢失计数，则认为i真的更丢了，更新 跟踪器状态，需要进行重定位了
            state = LOST;//更新 跟踪器状态
        }
        return;// 匹配点数量少，直接返回 ====================
    }
    
    vector<int> inlierIndex;
    Eigen::Isometry3d T = speed * lastPose;// 相对世界坐标系的位姿
    bool b = pnp->solvePnP( img, obj, currentFrame->camera, inlierIndex, T );
    if ( inlierIndex.size() < 15 ) // pnp 求解 内点数量过少，匹配效果也不好
    {
        cntLost ++;// 丢失计数===================
        if (cntLost > max_lost_frame)
        {
            state = LOST;
        }
        return;
    }
    
    currentFrame->setTransform( T );// 设置当前帧位姿态
    cntLost = 0;
    speed = T * lastPose.inverse();// 当前帧 到 世界坐标系 到 上一帧 ，两帧位姿差======
    lastPose = currentFrame->getTransform(); // 更新 上一帧的速度=============
    refFrames.push_back( currentFrame );     // 加入参考帧 队列
    
    while (refFrames.size() > refFramesSize ) // 查看参考帧队列容量，保持最近的几帧
    {
        refFrames.pop_front();// 丢弃 旧的=======
    }
    
    //cout<<"speed="<<endl<<speed.matrix()<<endl;
}

// 丢失后进行 重定位=============直接设置为上一参考帧位姿，后进行初始化===============
void    Tracker::lostRecover()
{
    cout<<"trying to recover from lost"<<endl;
    orb->detectFeatures( currentFrame );// 当前帧 特征检测
    currentFrame->setTransform( refFrames.back()->getTransform() );// 设置位姿为上一帧参考帧 的位姿
    refFrames.clear();// 参考帧队列清空
    refFrames.push_back( currentFrame );// 放入最新的一帧
    state = OK; // 相当于重新初始化了=========================
    cntLost = 0;
    cout<<"recover returned"<<endl;
}

```


> 跟踪线程测试代码
```c
#include "common_headers.h"
#include "rgbdframe.h"
#include "track.h"

using namespace std;
using namespace rgbd_tutor;

int main()
{
    cout<<"running tracker..."<<endl;
    ParameterReader para;
    FrameReader frameReader( para );
    Tracker tracker(para);

    while ( RGBDFrame::Ptr frame = frameReader.next() ) // 下一帧=======
    {
        cout<<"*************************************"<<endl;
        cout<<"tracking frame "<<frame->id<<endl;
        boost::timer    timer;
        Eigen::Isometry3d T = tracker.updateFrame( frame );// 当前帧跟踪参考帧
        cout<<"current T="<<endl<<T.matrix()<<endl;
        cv::imshow( "image", frame->rgb );
        if (tracker.getState() == Tracker::LOST)
        {
            cout<<"The tracker has lost"<<endl;
            cv::waitKey(0);
        }
        else
        {
            cv::waitKey(1);
        }
        cout<<"time cost = "<<timer.elapsed()<<endl;
    }
    return 0;
}

```



# 闭环检测 Looper.h

```c
#ifndef LOOPER_H
#define LOOPER_H

/**
  * Looper.h
  * The loop closure detector based on dbow2; 使用特征词带模型 进行闭环检测
  */

#include "common_headers.h"
#include "rgbdframe.h"
#include "converter.h"

#include <opencv2/opencv.hpp>

#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace rgbd_tutor
{
using namespace rgbd_tutor;

class Looper
{
public:

    Looper( const ParameterReader& para )
        : parameterReader( para )
    {
        string  vocab_file = para.getData<string>("looper_vocab_file"); // 特征词典=======
        cout<<"loading vocabulary file, this may take a while..."<<endl;
        vocab.loadFromTextFile( vocab_file );// 载入特征词典================
        cout<<"load ok."<<endl;

        min_sim_score = para.getData<float>("looper_min_sim_score");
        min_interval = para.getData<int>("looper_min_interval");
    }

    // 往闭环数据库里增加一条frame记录=======================================
    void add( RGBDFrame::Ptr& frame )
    {
        vector<cv::Mat> desps = frame->getAllDescriptorsVec(); // 输入帧的描述子
        DBoW2::FeatureVector featVec;
        vocab.transform( desps, frame->bowVec, featVec, 4);// 使用帧的 描述子 和特征词典 计算其 词典表示向量=======
        frames.push_back( frame );// 自身的一个 帧指针 数组vector
    }

    // 获取可能的loops=========================查找闭环==================
    vector<RGBDFrame::Ptr> getPossibleLoops( const RGBDFrame::Ptr& frame );
    // 返回 和当前帧具有相似性的 帧数组

    // TODO
    void save() {}
    void load() {}
protected:

    DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>    vocab;  //字典文件
    vector<RGBDFrame::Ptr>      frames; // 帧指针 数组vector
    const ParameterReader&      parameterReader;
    float   min_sim_score   =0.01; // 相似性u阈值====
    float   min_interval    =10;   // 闭环 帧 id差 阈值====
};

}
#endif // LOOPER_H


```

> 闭环检测实现 Looper.c
```c
#include "looper.h"

using namespace rgbd_tutor;
// 获取可能的loops=========================查找闭环==================
vector<RGBDFrame::Ptr> Looper::getPossibleLoops( const RGBDFrame::Ptr& frame )
{
    vector<RGBDFrame::Ptr>  result;
    for ( size_t i=0; i<frames.size(); i++ )// 遍例闭环数据库 帧
    {
        RGBDFrame::Ptr pf = frames[i];
        double  score = vocab.score( frame->bowVec, pf->bowVec );// 计算所给帧 和 数据库 帧 的相似性
        if (score > min_sim_score && abs(pf->id-frame->id)>min_interval )// 比较相似 且 距离较远
        {
            result.push_back( pf );// 与所给帧较相似==========
        }
    }
    return result;
}

```

# 位姿图 优化 后端   全局优化===========
pose_graph.h
```c
// 
#ifndef POSE_GRAPH_H
#define POSE_GRAPH_H
#include "common_headers.h"
#include "parameter_reader.h"
#include "rgbdframe.h"
#include "looper.h" // 闭环检测
#include "pnp.h"
#include "orb.h"
#include "track.h" // 跟踪器

#include <thread>  // 线程
#include <mutex>   // 锁
#include <functional>
#include <map>
#include <condition_variable> // 锁变量

/**
  * The pose graph performs global optimization.
  * 在tracker跟踪当前帧currentFrame，返回一个粗略的位姿时，会把这个帧尝试放入pose graph中.
  * pose graph将它 与 自己的refframe比较，得到一个相对位移估计。
         当这个估计大于给定阈值时，将该帧作为新的frame插入到pose graph中。
  * pose graph本身有一个优化线程，负责查找相近节点的边并进行优化。
  * 同时它有回环检测模块。检测到大型回环时进行全局优化。
  * 
  * 进行优化时，关键帧序列的位姿会被替换成优化后的值，因此参考帧会发生改变。而tracker存在漂移。
  * 为了使机器人得到全局准确的位姿，需要用pose graph优化后的结果校正tracker。
  */

namespace rgbd_tutor
{
using namespace rgbd_tutor;

class Tracker;// 声明跟踪类

class PoseGraph
{
public:
    PoseGraph( const ParameterReader& para,
               shared_ptr<Tracker>& t )
        : parameterReader( para ),
          tracker( t )
    {
        looper  =   make_shared<Looper>( para );          // 初始化 闭环检测器
        orb     =   make_shared<OrbFeature>( para );      // 初始化 orb特征检测器
        pnp     =   make_shared<PnPSolver>( para, *orb ); // 初始化 2d-3d 位姿变换 球解器

// 关键帧选取准则
        keyframe_min_translation = para.getData<double>("keyframe_min_translation");//最小平移量
        keyframe_min_rotation = para.getData<double>("keyframe_min_rotation");// 最小旋转量
        nearbyFrames = para.getData<int>("nearby_keyframes");// 
    
        loopAccuError = para.getData<double>("loop_accumulate_error");  // 闭环全局累计误差              
        localAccuError = para.getData<double>("local_accumulate_error");// 局部累计误差   

// 多线程 绑定 mainLoop 函数=============================!!!!!!!!!!!!!!!!!!!!!!!!!!!============
        posegraphThread = make_shared<std::thread> (
                    std::bind(&PoseGraph::mainLoop, this) );
		    
// 初始化 g2o 图优化对象===================
        g2o::LinearSolverCholmod<g2o::BlockSolver<g2o::BlockSolverTraits<6,3> >::PoseMatrixType > * linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver<g2o::BlockSolverTraits<6,3> >::PoseMatrixType > ();

        //g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType> ();
        linearSolver->setBlockOrdering( false );

        g2o::BlockSolver< g2o::BlockSolverTraits<6,3> >* solver = new g2o::BlockSolver< g2o::BlockSolverTraits<6,3> >( linearSolver );
        //g2o::BlockSolver_6_3*   solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg* algo = new g2o::OptimizationAlgorithmLevenberg( solver );
        optimizer.setAlgorithm( algo );
        optimizer.setVerbose(false);

    }

    // 试图插入一个新的keyframe，失败返回false====================
    bool    tryInsertKeyFrame( RGBDFrame::Ptr& frame );

    // 主线程===============================
    void    mainLoop();
    
    // 关闭，清理工作========================
    void    shutdown()
    {
        shutDownFlag = true;
	
        keyframe_updated.notify_all(); // 通知 关键帧消费者
	
        cout<<"please wait pose graph thread to stop..."<<endl;
        if (posegraphThread != nullptr)
        {
            posegraphThread->join();// 等待图优化线程 结束  join() 
        }

        // save the results  保存结果========
        cout<<RED<<"saving trajectory"<<endl;
        cout<<"vertex: "<<optimizer.vertices().size()<<endl;// 定点数量
        cout<<"edges: "<<optimizer.edges().size()<<endl;    // 边数量
        save( "./data/traj.g2o" );
        cout<<"trajectory ok."<<RESET<<endl;

        if ( optimizer.vertices().size() > 5)
        {
            // 太少了就不优化了
            optimizer.initializeOptimization();
            optimizer.optimize( 10 );
        }
    }

// 保存关键帧========================================
    void save( const string& filename )
    {
        ofstream fout(filename);
        /*
        for ( size_t i=0; i< keyframes.size(); i++ )
        {
	// 按 帧id 从图中 获取对应 顶点=========================
            g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*> (optimizer.vertices()[ keyframes[i]->id ]);
            if (v == nullptr)
            {
                cerr<<"vertex "<<keyframes[i]->id<<" does not exist!"<<endl;
                continue;
            }
            double data[7] = {0};// 对应的位姿 x,y,z,qx,qy,qz,w
            v->getEstimateData( data );// 存储到 data内=========
            fout<<"VERTEX_SE3:QUAT "<<v->id()<<" ";
            for ( double d:data )
                fout<<d<<" ";
            fout<<endl;
        }
      //  边==============================================
        for ( size_t i=0; i<edges.size(); i++ )
        {
            g2o::EdgeSE3 edge = *edges[i];
            fout<<"EDGE_SE3:QUAT "<<edge.vertices()[0]->id()<<" "<<edge.vertices()[1]->id()<<" ";
            double data[7] = {0};
            edge.getMeasurementData( data ); // 位姿 差=========================
            for ( double d:data )
                fout<<d<<" ";
            fout<<"100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 100 0 0 100 0 100"<<endl;
        }
        */
        if (fout)
        {
            optimizer.save(filename.c_str());
            fout.close();
        }
    }

protected:
    //  检测 限定端点的边 是否存在
    bool    isEdgeExist( const int vertex1, const int vertex2 ) const
    {
        if ( vertex1==vertex2 )
            return true;
        EdgeID e1, e2;
        e1[vertex1] = vertex2; // 两个 顶点(不同id) 之前的 双向连线=========
	e2[vertex2]=vertex1;
        return edges.find( e1 ) != edges.end() ||
                edges.find( e2 ) != edges.end() ;//  map中查找 边=======
    }

public:
    //数据
    vector<RGBDFrame::Ptr>  keyframes;      // 关键帧====
    vector<RGBDFrame::Ptr>  newFrames;      // 新关键帧的缓冲区
    RGBDFrame::Ptr          refFrame;       // 参考
    
    
// 互斥锁 std::mutex是一种最常见的线程间同步的手段，但是在有些情况下不太高
// https://www.jianshu.com/p/c1dfa1d40f53  
// 互斥锁 + 延时，可减小cpu占用率
// 然后困难之处在于，如何确定这个延时时间呢，假如生产者生产的很快，消费者却延时500ms，也不是很好。
// 这就引出了条件变量（condition variable）
// 其中有两个重要的接口，notify_one()和wait()，wait()可以让线程陷入休眠状态，
// 如果生产者发现队列中没有东西，就可以让自己休眠，但是不能一直不干活啊，notify_one()就是唤醒处于wait中的其中一个条件变量。
// 那什么时刻使用 notify_one() 比较好呢，当然是在生产者往队列中放数据的时候了，队列中有数据，就可以赶紧叫醒等待中的线程起来干活了。
// 消费者 wait() 等待 生产折的数据
// 生产者 产生了数据 通知 notify_one()  消费者 来 消费  类似 双11 活动!!!!!!!!!!
// 除了notify_one()函数，c++还提供了notify_all()函数，可以同时唤醒所有处于wait状态的条件变量。

    std::condition_variable keyframe_updated; // 数据 好没好 条件变量============
    std::mutex              keyframe_updated_mutex;  // 关键帧更新锁============
    
    bool                    shutDownFlag    =false;
    std::mutex              keyframes_mutex; // 关键帧 锁======================

    // 有用的loop之类的东西
    shared_ptr<Looper>      looper      =nullptr;// 闭环
    shared_ptr<PnPSolver>   pnp         =nullptr;// 2d-3d 位姿变换 g2o求解
    shared_ptr<OrbFeature>  orb         =nullptr;// orb特征求解器

    // pose graph 线程
    shared_ptr<std::thread> posegraphThread =nullptr; // 执行 mainLoop() 函数 主线程======

    // tracker 调整当前的姿态
    shared_ptr<Tracker>     tracker     =nullptr;// 跟踪器

    // g2o的优化器
    g2o::SparseOptimizer    optimizer;

    // 点集与边集
    vector<int>             vertexIdx;// 定点id  帧 id
    typedef map<int,int>    EdgeID;  // 边 id  两帧 之间 ()
    map< EdgeID, g2o::EdgeSE3* > edges;// 边 map 字典 

    // 参数读取类
    const   ParameterReader&    parameterReader;

    double  keyframe_min_translation        =0.3; // 关键帧阈值
    double  keyframe_min_rotation           =0.3;
    int     nearbyFrames                    =2;
    int     lastGraphSize                   =0;
    double  loopAccuError                   =1.0;// 闭环误差
    double  localAccuError                  =1.0;// 局部优化BA 误差

};

}
#endif // POSE_GRAPH_H


```

> 回环优化 全局优化 实现 pose_graph.c

```c
#include "pose_graph.h"
#include "utils.h"
using namespace rgbd_tutor;

/**
 * @brief PoseGraph::tryInsertKeyFrame
 * @param frame
 * @return
 * TODO: Problem: when inserting large number of keyframes,
                  the mapping thread will block for long time, 
		  causing even more key-frames to be processed.
 */
 
// 试图插入一个新的keyframe，失败返回false====================
bool PoseGraph::tryInsertKeyFrame(RGBDFrame::Ptr& frame)
{
    if ( keyframes.size() == 0 )
    {
        // 图是空的，直接加入原始点
        unique_lock<mutex> lck(keyframes_mutex);// 关键帧 上锁
        keyframes.push_back(frame);// 加入 关键帧队列
        refFrame = frame;          // 设置为参考帧
        g2o::VertexSE3* v = new g2o::VertexSE3();// 新顶点
        v->setId( frame->id );                   // 顶点id
        v->setEstimate( frame->T_f_w.inverse() );// 估计值
        v->setFixed(true);                       // 第一帧位姿 固定 
        optimizer.addVertex( v );                // 优化器中加入顶点
        vertexIdx.push_back( frame->id );        // 顶点 集合 记录
        return true;
    }

    // 计算 frame 和 refFrame 之间的位移差
    Eigen::Isometry3d delta = frame->getTransform().inverse() * refFrame->getTransform();// 当前帧和参考帧 位姿
    // 位姿变化足够大，才加入 关键帧====================================
    if ( norm_translate( delta ) > keyframe_min_translation ||
         norm_rotate( delta ) > keyframe_min_rotation )
    {
        // 离keyframe够远
        // 在key frames中进行插入，并在图中生成对应节点和边
        unique_lock<mutex> lck(keyframes_mutex);
        cout<<YELLOW<<"adding keyframe "<<frame->id<<" with ref to "<<refFrame->id<<", n_t="<<norm_translate( delta )<<",n_r="<<norm_rotate(delta)<<RESET<<endl;
        newFrames.push_back( frame );            // 加入 新关键帧 序列=================
        
        //  add the vertex
        g2o::VertexSE3* v = new g2o::VertexSE3();// 新顶点
        v->setId( frame->id );                   // 顶点id
        v->setEstimate( frame->getTransform().inverse() );// 估计值
        v->setFixed(false);                               // 进行优化
        optimizer.addVertex( v );                         // 优化器中加入顶点
        vertexIdx.push_back( frame->id );                 // 顶点 集合 记录
        keyframes.push_back( frame );                     // 加入  关键帧 序列=================

        // and the edge with refframe
        // 这里直接根据refFrame和currentFrame的位姿差生成一个边
        // 因为位姿差是tracker估计出来的，我们认为这是比较准的
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 注意边的赋值有些绕，详见EdgeSE3的误差计算方式
        g2o::VertexSE3* v0 = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( refFrame->id ));// 参考帧id 对应的 顶点
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( frame->id ));   // 当前帧id 对应的 顶点
        edge->setVertex(0, v1);// 边两个端点，端点0，端点1=================
        edge->setVertex(1, v0);
        // because the state is estimated from tracker
        edge->setMeasurementFromState();
        edge->setInformation( Eigen::Matrix<double,6,6>::Identity() * 100);
        edge->setRobustKernel( new g2o::RobustKernelHuber() );

        EdgeID id;// 边id 是个map<int,int> 两个顶点的id
        id[refFrame->id] = frame->id;// 
        edges[ id ] = edge;//  记录 边=================================
        optimizer.addEdge( edge );
        
        // set ref frame to current
        refFrame = frame;// 更新参考帧=================

        keyframe_updated.notify_one();// 参考帧更新完毕，其他线程可以用了===========
        return true;
    }
    else
    {
        return false;// 不需要添加关键帧=====
    }
}

/**
 * @brief PoseGraph::mainLoop  位姿 图优化 主线程===============
 */
void PoseGraph::mainLoop()
{
    cout<<"starting pose graph thread..."<<endl;
    double  loopAccumulatedError = 0.0;  // 全局 回环的累积误差
    double  localAccumulatedError = 0.0; // 局部 回环的累积误差
    while(1)
    {
        if (shutDownFlag == true)// 关闭了
        {
            break;
        }
        unique_lock<mutex> lck_update_keyframe(keyframe_updated_mutex);// 关键帧更新 锁
        keyframe_updated.wait( lck_update_keyframe );// 等到keyframes有更新==============
        cout<<"keyframes are updated"<<endl;
        boost::timer timer;
        // 复制一份newFrames，防止处理的时候有新的东西插入=======
        unique_lock<mutex> lck(keyframes_mutex);
        vector<RGBDFrame::Ptr>  newFrames_copy = newFrames;
        newFrames.clear();
        

        bool    findLargeLoop = false;
        // 检测新增的keyframe并检测其中的回环
        // 边
        cout<<"new key frames = "<<newFrames_copy.size()<<endl;
        
        for ( auto nf : newFrames_copy )// 新增加的 关键帧=========
        {
	
	// 附近关键帧 小回环检测===========局部滑窗============================
            // 检测nf和keyframes末尾几个的关系
            // 请注意 事实上 neframes 里的东西已经出现在 keyframes 里边了
            for ( int i=0; i<nearbyFrames; i++ )// 附近的关键帧
            {
                int idx = keyframes.size()-i-2;// 两帧之前的几个关键帧
                if (idx < 0)
                {
                    break;
                }
                RGBDFrame::Ptr pf = keyframes[idx];
                //cout<<"checking "<<nf->id<<" and "<<pf->id<<endl;
                //  检测边是否存在
                if (isEdgeExist( nf->id, pf->id ))
                {
                    continue;// 边已经存在，跳过========
                }

                // 用pnp检测nf和pf之间是否可以计算一个边
                PNP_INFORMATION info;
                if ( pnp->solvePnPLazy( pf, nf, info, false ) == false )
                {
                    continue;// 不能形成边，返回========
                }
                // pnp成功，将pnp结果加到graph中====================================================
                cout<<"solve pnp ok, generating an edge"<<endl;
                g2o::EdgeSE3* edge = new g2o::EdgeSE3();
		// 边两个端点，端点0，端点1=================
                edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( nf->id ));
                edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( pf->id ));
                edge->setMeasurement( info.T );// pnp计算出来的位姿
                edge->setInformation( Eigen::Matrix<double,6,6>::Identity() * 100);
                edge->setRobustKernel( new g2o::RobustKernelHuber() );
                
                edge->computeError();
                cout<<"add local error "<< edge->chi2() <<endl;
                localAccumulatedError += edge->chi2();// 局部回环 误差累积 ===============
		
                EdgeID id;
                id[nf->id] = pf->id;
                edges[ id ] = edge;// 记录 边==================
		
                optimizer.addEdge( edge );
                cout<<"edge has been added"<<endl;
            }// end of for nearbyFrames

         // 利用词典模型进行大回环检测=====================================================
            looper->add( nf );
            vector<RGBDFrame::Ptr>  possibleLoops = looper->getPossibleLoops( nf );//可能的 回环帧=====

            for ( auto pf:possibleLoops )// 遍历 每一个 可能的 回环帧==
            {
                if ( isEdgeExist( nf->id, pf->id ) ) //这条边已经存在
                    continue;
                PNP_INFORMATION info;
		// 用pnp检测nf和pf之间是否可以计算一个边
                if ( pnp->solvePnPLazy( pf, nf, info, false) == true )
                {
                    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
		    // 边两个端点，端点0，端点1=================
                    edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( nf->id ));
                    edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( pf->id ));
                    edge->setMeasurement( info.T );// pnp计算出来的位姿
                    edge->setInformation( Eigen::Matrix<double,6,6>::Identity() * 100);
                    edge->setRobustKernel( new g2o::RobustKernelHuber() );
                    //edges.push_back( edge );
                    EdgeID id;
                    id[nf->id] = pf->id;
                    edges[ id ] = edge;// 记录 边=========
                    optimizer.addEdge( edge );

                    edge->computeError();
                    loopAccumulatedError += edge->chi2();// 大回环 误差=============
                }
            } // end of for possible loops
        } // end of for new frames

        // 处理优化
        bool doOptimize = false;
	// 大回环误差过大，进行优化============================
        if ( loopAccumulatedError > loopAccuError )
        {
            // 处理全局优化
            for ( auto v:vertexIdx )
            {
                optimizer.vertex(v)->setFixed(false);// 除去 第一个顶点外全部进行优化
            }
            optimizer.vertex( vertexIdx[0] )->setFixed(true);// 第一个 顶点固定
            cout<<"global optimization"<<endl;
            optimizer.initializeOptimization();// 全局优化========
            optimizer.optimize(10);
            // 重置keyframes和refFrame
            for ( auto kf : keyframes )// 从g2o优化图中 更新 关键帧
            {
                g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*> ( optimizer.vertex( kf->id ) );
                if ( v )
                {
                    kf->setTransform( v->estimate().inverse());// 使用图优化后的 结果 更新 关键帧
                }
            }
            localAccumulatedError = 0; // 重置误差记录======
            loopAccumulatedError  = 0;
            doOptimize = true;
        }
	// 反之小回环误差过大
        else if ( localAccumulatedError > localAccuError )
        {
            // 处理局部优化
            for ( auto v:vertexIdx )
            {
                optimizer.vertex( v )->setFixed( true );// 仅仅优化最近5帧，其他帧不优化
            }
            for ( int i=vertexIdx.size()-1; i>0 && i>vertexIdx.size()-6; i-- )//
            {
                optimizer.vertex( vertexIdx[i] )->setFixed( false );// 仅仅优化最近5帧
            }
            optimizer.vertex( vertexIdx[0] )->setFixed(true);// 第一个 顶点固定
            cout<<"local optimization"<<endl;
            
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            // 重置
            for ( int i=keyframes.size()-1; i>0 && i>keyframes.size()-6; i-- )// 更新 局部优化的5帧===
            {
                //cout<<"i="<<i<<", keyframe size="<<keyframes.size()<<endl;
                g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*> ( optimizer.vertex( keyframes[i]->id ) );
                if ( v )
                {
                    keyframes[i]->setTransform( v->estimate().inverse() );// 使用图优化后的 结果 更新 关键帧
                }
                
            }
            localAccumulatedError = 0; // 重置 局部回环误差记录
            doOptimize = true;
        } // end of if loop accu
        
        if ( doOptimize == true )// 优化完成
        {
            refFrame = keyframes.back();// 最后的关键帧
            tracker->adjust( refFrame );// 使用最近的关键帧更新 跟踪器
        }
    }
    cout<<"pose graph thread stops"<<endl;
}


```


> 全局 图优化 测试代码=====
```c
#include "track.h"
#include "pose_graph.h"
using namespace rgbd_tutor;

int main()
{
    ParameterReader	parameterReader;
    Tracker::Ptr	tracker( new Tracker(parameterReader) );// 跟踪器指针=====
    FrameReader		frameReader( parameterReader );
    PoseGraph		poseGraph( parameterReader, tracker );

    while( RGBDFrame::Ptr frame = frameReader.next() )
    {
        cout<<"*******************************************"<<endl;
        boost::timer timer;
        cout<<"loading frame "<<frame->id<<endl;
        Eigen::Isometry3d T = tracker->updateFrame( frame );// 跟踪器 进行跟踪=====
        cout<<"current frame T = "<<endl<<T.matrix()<<endl; 
        cv::imshow( "image", frame->rgb );
        if ( poseGraph.tryInsertKeyFrame( frame ) == true )// 在图中插入一帧========
        {
            cout<<"Insert key-frame succeed"<<endl;
            cv::waitKey(1);
        }
        else
        {
            cout<<"Insert key-frame failed"<<endl;
            cv::waitKey(1);
        }
        cout<<GREEN<<"time cost="<<timer.elapsed()<<RESET<<endl;
    }

    poseGraph.shutdown();
    return 0;
}


```



# 地图构建
> mapper.h
```
#ifndef MAPPER_H
#define MAPPER_H

#include "common_headers.h"
#include "rgbdframe.h"
#include "pose_graph.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

namespace rgbd_tutor
{
using namespace rgbd_tutor;

class Mapper
{
public:
    typedef pcl::PointXYZRGBA PointT; // 点
    typedef pcl::PointCloud<PointT> PointCloud; // 点云

    Mapper( const ParameterReader& para, PoseGraph& graph )
        : parameterReader( para ), poseGraph( graph )
    {
        resolution = para.getData<double>("mapper_resolution");   // 点云精度
        max_distance = para.getData<double>("mapper_max_distance");// 

        viewerThread = make_shared<thread> ( bind( &Mapper::viewer, this ));// 可视化线程======
    }
    void shutdown()
    {
        shutdownFlag = true;
        viewerThread->join();// 等待 可视化线程结束
    }

    // viewer线程
    void viewer();

protected:
    PointCloud::Ptr generatePointCloud( const RGBDFrame::Ptr& frame );// 生成点云========

protected:
    // viewer thread
    shared_ptr<thread>		viewerThread = nullptr;// 可视化线程指针=====
    const ParameterReader& parameterReader;
    PoseGraph&  poseGraph;// 图优化
    
    // 设置默认值
    int    keyframe_size    = 0;
    double resolution       = 0.02;
    double max_distance     = 8.0;
    bool	shutdownFlag	= false;

};


}

#endif // MAPPER_H

```

> mapper.c

```c
#include "mapper.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>// 体素格滤波======

using namespace rgbd_tutor;

Mapper::PointCloud::Ptr Mapper::generatePointCloud(const RGBDFrame::Ptr &frame)
{
    PointCloud::Ptr tmp( new PointCloud() );
    if ( frame->pointcloud == nullptr )
    {
        // point cloud is null ptr
        frame->pointcloud = boost::make_shared<PointCloud>();


// omp 多线程 并行处理 生成点云=================================!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#pragma omp parallel for
        for ( int m=0; m<frame->depth.rows; m+=3 ) // 间隔3行生成点云=========
        {
            for ( int n=0; n<frame->depth.cols; n+=3 )
            {
                ushort d = frame->depth.ptr<ushort>(m)[n];// 深度值
                if (d == 0)
                    continue;
                if (d > max_distance * frame->camera.scale)// 深度值过大 不可靠
                    continue;
                PointT p;
                cv::Point3f p_cv = frame->project2dTo3d(n, m);// 2d点投影成3d点
                p.b = frame->rgb.ptr<uchar>(m)[n*3];
                p.g = frame->rgb.ptr<uchar>(m)[n*3+1];
                p.r = frame->rgb.ptr<uchar>(m)[n*3+2];

                p.x = p_cv.x;
                p.y = p_cv.y;
                p.z = p_cv.z;

                frame->pointcloud->points.push_back( p );
            }
        }
    }

    Eigen::Isometry3d T = frame->getTransform().inverse();
    pcl::transformPointCloud( *frame->pointcloud, *tmp, T.matrix());// 当前帧点云转换到 世界坐标系下
    tmp->is_dense = false;// 非稠密点云，有nan值
    return tmp;
}


// 可视化线程执行的函数======================================
void Mapper::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    PointCloud::Ptr globalMap (new PointCloud);
    
// 体素格滤波参数==============================
    pcl::VoxelGrid<PointT>	voxel;
    voxel.setLeafSize( resolution, resolution, resolution );

    while (shutdownFlag == false)
    {
        static int cntGlobalUpdate = 0;
        if ( poseGraph.keyframes.size() <= this->keyframe_size )
        {
            usleep(1000); // 图优化线程  慢了 等待 ===========================
            continue;
        }
        // keyframe is updated
        PointCloud::Ptr	tmp(new PointCloud());
        if (cntGlobalUpdate % 15 == 0) // 每隔15 次 更新地图
        {
            // update all frames
            cout<<"redrawing frames"<<endl;
            globalMap->clear();// 地图清空===============
            for ( int i=0; i<poseGraph.keyframes.size(); i+=2 )// 间隔两帧生成点云地图============
            {
                PointCloud::Ptr cloud = this->generatePointCloud(poseGraph.keyframes[i]);
                *globalMap += *cloud;
            }
        }
        else
        {
            for ( int i=poseGraph.keyframes.size()-1; i>=0 && i>poseGraph.keyframes.size()-6; i-- )// 最近5帧点云加入到地图=====
            {
                PointCloud::Ptr cloud = this->generatePointCloud(poseGraph.keyframes[i]);
                *globalMap += *cloud;
            }
        }

        cntGlobalUpdate ++ ;
        //voxel
        voxel.setInputCloud( globalMap );
        voxel.filter( *tmp );

        keyframe_size = poseGraph.keyframes.size(); // 更新 关键帧数量
        globalMap->swap( *tmp ); // 滤波
        viewer.showCloud( globalMap );

        cout<<"points in global map: "<<globalMap->points.size()<<endl;
    }
}


```


> 地图构建测试
```c
#include "rgbdframe.h"
#include "track.h"
#include "pose_graph.h"
#include "mapper.h"
#include "common_headers.h"


using namespace std;
using namespace rgbd_tutor;
int main()
{
    ParameterReader	parameterReader;
    Tracker::Ptr	tracker( new Tracker(parameterReader) ); // 跟踪器指针
    FrameReader		frameReader( parameterReader );
    PoseGraph		poseGraph( parameterReader, tracker );   // 图优化
    Mapper			mapper( parameterReader, poseGraph );// 建图

    while ( RGBDFrame::Ptr frame = frameReader.next() )
    {
        boost::timer timer;
        cv::imshow("image", frame->rgb);
        cv::waitKey(1);
        tracker->updateFrame( frame ); // 跟踪
        poseGraph.tryInsertKeyFrame( frame );// 图优化插入
        
        if (tracker->getState() == Tracker::LOST)
        {
            cout<<"tracker is lost"<<endl;
            //break;
        }
        cout<<"cost time = "<<timer.elapsed()<<endl;
    }

    poseGraph.shutdown();
    mapper.shutdown();

    return 0;
}

```






