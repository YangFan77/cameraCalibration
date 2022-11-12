# cameraCalibration
标定
该文件为cmake文件
在文件夹下操作
mkdir build
cd build
cmake ..
make
就会在文件夹目录生成一个bin文件夹
在bin目录下创建四个文件夹 分别为 3d imgs remap result
remap下再次创建3d文件夹 
imgs放入标定图像
result下就会出现标出角点的文件
3d文件夹可以放置可不放 该文件夹下的图片将会按照标定的畸变自动校正畸变 存在remap/3d中
可根据代码修改  如有问题联系邮箱 770267939@qq.com
