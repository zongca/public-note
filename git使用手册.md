# git的使用笔记



教程网址：https://www.runoob.com/git/git-tutorial.html



## 一、Git简介

git是一种开源的分布式版本控制系统，可以有效高速的处理项目版本管理。

## 二、Git的常用概念



## 三、Git的使用

### git配置

#### 1、配置用户信息

配置个人的用户名称和电子邮件地址：

```
git config --global user.name "runoob"
git config --global user.email test@runoob.com
```

#### 2、配置文本编辑器

设置Git默认使用的文本编辑器, 一般可能会是 Vi 或者 Vim。如果你有其他偏好，比如 Emacs 的话，可以重新设置：

```
 git config --global core.editor emacs
```

#### 3、差异分析工具

在解决合并冲突时使用哪种差异分析工具。比如要改用 vimdiff 的话：

```
git config --global merge.tool vimdiff
```

#### 4、查看配置信息

要检查已有的配置信息，可以使用 git config --list 命令：

```
git config --list
```

也可以直接查阅某个环境变量的设定，只要把特定的名字跟在后面即可，像这样：

```
git config user.name
```

### 与Github仓库绑定

github 简明教程：https://www.runoob.com/w3cnote/git-guide.html





使用步骤：

1、git init

2、设置username和email

git config --global user.name "DLUT_ZCA"

git config --global user.email "864224983@qq.com"

3、添加远程地址

git remote add origin git@github.com:zongca/Cplusplus.git

4、上传文件

（1）添加需要上传的文件

git add filename

（2）commit 

git commit -m "代码提交信息"

（3）上传到远端服务器

git push origin master

