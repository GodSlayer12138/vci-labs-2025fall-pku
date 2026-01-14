# Final Project
## 选题
### Practical Pigment Mixing for Digital Painting (⭐⭐)
![](https://vcl.pku.edu.cn/course/vci/labs/project-A-3.png)

这是一篇 SIGGRAPH 2021 上的 [论文](https://scrtwpns.com/mixbox.pdf) 。作者注意到在现在的画图软件中，颜色的混合并没有按照颜料混合的规律。比如上图中，蓝色和黄色的颜料混合应该出现绿色，但是各种专业的绘图软件都没有混合出绿色。作者提出的了一种简单的方法在 RGB 的颜色空间中按照颜料的方式混合颜色的方法，并提供了 [开源代码](https://github.com/scrtwpns/mixbox) 。你可以首先阅读论文和对应的讲解视频理解算法的原理，然后在 lab 的代码基础上复现出来，并提供工具来使用这个混合方法创造画作！