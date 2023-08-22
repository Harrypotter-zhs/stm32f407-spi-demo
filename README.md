# SPI传输44K字节的数据内容，需要进行分包进行传输

## 现在进度

已经完成的分包发送并且数据不足固定字节数据补0发送



git 命令上传版本库

```shell
# 创建一个新的仓库
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin git@github.com:Harrypotter-zhs/stm32f407-spi-demo.git
git push -u origin main
# 向一个仓库提交代码
git remote add origin git@github.com:Harrypotter-zhs/stm32f407-spi-demo.git
git branch -M main
git push -u origin main

```

## 注意事项

```shell
Argon:
你这个分包没补包吧

Argon:
最后一包不满足长度怎么办

Argon:
做定长好处就是后面有问题用逻辑分析仪抓的时候一下就看出来那个包长度不对了

Argon:
不定长，最后一包不一样长度。到时候逻辑分析花花绿绿的很难看

飞翔的鸡大保:
现在没补包机制

Argon:
另外还有接受函数

Argon:
你这个接受才是最容易处bug的

Argon:
还有包头的设计

Argon:
你一块一块的写

Argon:
先写数据发送


Argon:
就只有分包机制的

Argon:
然后再写包头解析和构建

Argon:
每个都单独测试

Argon:
最后合并再一起

Argon:
再在上面封装协议接口

Argon:
不要都写完再测

Argon:
到时候问题一堆


Argon:
分包写完就一个数据发送的压测下

Argon:
压测一天

Argon:
就是循环发数据0到255

Argon:
错包打印一下

飞翔的鸡大保:
哦哦，发的数据我都知道，看哪一个错误打印出来

Argon:
压测个72小时

Argon:
没问题就没啥问题了

Argon:
做好单元测试

Argon:
你这其实都可以在linux上把数据处理封装好

Argon:
再再板子上盘

Argon:
linux直接socket

Argon:
把你的接口压测起来


Argon:
都行看你的

飞翔的鸡大保:
嗯嗯，

Argon:
我给你说的让你压测的是你的传输协议

Argon:
你自己写的纯软件的

Argon:
和平台没啥关系

Argon:
要不你需要有好几个板子主机从机一对一对的

Argon:
最少4块

Argon:
挂机一块调一块

Argon:
还得有个电脑在那里挂串口日志
```
