# Github的使用手册

参考网址：https://www.runoob.com/w3cnote/git-guide.html



## Github的定义

github是一个基于git的代码托管平台，付费用户可以建私人仓库，我们一般的免费用户只能使用公共仓库，也就是代码要公开。



## Github的使用

### 配置Git

1、首先在本地闯将ssh key：

```
ssh-keygen -t rsa -C "your_email@youremail.com"
```

后面的`your_email@youremail.com`（注意保留双引号）改为你在github上注册的邮箱，之后会要求确认路径和输入密码，我们这使用默认的一路回车就行。成功的话会在`~/`下生成`.ssh`文件夹，进去，打开`id_rsa.pub`，复制里面的`key`。

2、github上面的配置

回到github上，进入 Account Settings（账户配置），左边选择SSH Keys，Add SSH Key,title随便填，粘贴在你电脑上生成的key。

3、验证是否成功

为了验证是否成功，在git bash下输入：

```
 ssh -T git@github.com
```

4、上传本地仓库

（1）设置username和email

```
git config --global user.name "DLUT_ZCA"
```

```
git config --global user.email "864224983@qq.com"
```

（2）添加远程地址

进入要上传的仓库，右键git bash，添加远程地址：

```
git remote add origin git@github.com:zongca/Cplusplus.git
```

后面的yourName和yourRepo表示你再github的用户名和刚才新建的仓库，加完之后进入.git，打开config，这里会多出一个remote "origin"内容，这就是刚才添加的远程地址，也可以直接修改config来配置远程地址。

创建新文件夹，打开，然后执行` git init` 以创建新的 git 仓库。

5、检出仓库

执行如下命令以创建一个本地仓库的克隆版本：

```
git clone /path/to/repository 
```

如果是远端服务器上的仓库，你的命令会是这个样子：

```
git clone username@host:/path/to/repository
```

### Git概念

#### 1、工作流

本地仓库由 git 维护的三棵"树"组成。

第一个是你的 `工作目录`，它持有实际文件；

第二个是 `暂存区（Index）`，它像个缓存区域，临时保存你的改动；

第三个是 `HEAD`，它指向你最后一次提交的结果。

可以提出更改（把它们添加到暂存区），使用如下命令：

git add <filename>

git add *

这是 git 基本工作流程的第一步；使用如下命令以实际提交改动：

git commit -m "代码提交信息"

现在，你的改动已经提交到了 **HEAD**，但是还没到你的远端仓库。

![image-20220422234903225](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20220422234903225.png)

#### 2、推送改动

你的改动现在已经在本地仓库的 **HEAD** 中了。执行如下命令以将这些改动提交到远端仓库：

git push origin master

可以把 *master* 换成你想要推送的任何分支。

#### 3、本地仓库连接远程服务器

如果你还没有克隆现有仓库，并欲将你的仓库连接到某个远程服务器，你可以使用如下命令添加：

git remote add origin <server>

如此你就能够将你的改动推送到所添加的服务器上去了。

#### 4、分支

git中分支的理解：https://m.php.cn/tool/git/486747.html

分支是用来将特性开发绝缘开来的。在你创建仓库的时候，*master* 是"默认的"分支。在其他分支上进行开发，完成后再将它们合并到主分支上。

![image-20220423094928223](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20220423094928223.png)

也就是说：分支是用来完成一些特定的任务开发的，其不影响主分支的进行，在分支开发完毕后可以合并到主分支。

创建一个叫做"feature_x"的分支，并切换过去：
`git checkout -b feature_x`
切换回主分支：
`git checkout master`
再把新建的分支删掉：
`git branch -d feature_x`
除非你将分支推送到远端仓库，不然该分支就是 *不为他人所见的*：
`git push origin <branch>`

#### 5、更新与合并

要更新你的本地仓库至最新改动，执行：
`git pull`
以在你的工作目录中 *获取（fetch）* 并 *合并（merge）* 远端的改动。

要合并其他分支到你的当前分支（例如 master），执行：
`git merge <branch>`

在这两种情况下，git 都会尝试去自动合并改动。遗憾的是，这可能并非每次都成功，并可能出现*冲突（conflicts）*。 这时候就需要你修改这些文件来手动合并这些*冲突（conflicts）*。改完之后，你需要执行如下命令以将它们标记为合并成功：
`git add <filename>`
在合并改动之前，你可以使用如下命令预览差异：
`git diff <source_branch> <target_branch>`

#### 6、标签

为软件发布创建标签是推荐的。这个概念早已存在，在 SVN 中也有。你可以执行如下命令创建一个叫做 *1.0.0* 的标签：
`git tag 1.0.0 1b2e1d63ff`
*1b2e1d63ff* 是你想要标记的提交 ID 的前 10 位字符。可以使用下列命令获取提交 ID：
`git log`
你也可以使用少一点的提交 ID 前几位，只要它的指向具有唯一性。

##### 7、替换本地改动

假如你操作失误（当然，这最好永远不要发生），你可以使用如下命令替换掉本地改动：
`git checkout -- <filename>`
此命令会使用 HEAD 中的最新内容替换掉你的工作目录中的文件。已添加到暂存区的改动以及新文件都不会受到影响。

假如你想丢弃你在本地的所有改动与提交，可以到服务器上获取最新的版本历史，并将你本地主分支指向它：
`git fetch origin`
`git reset --hard origin/master`

