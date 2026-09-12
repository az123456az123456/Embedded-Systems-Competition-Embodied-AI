# 团队 Git 协作规范

> 适用：Embedded-Systems-Competition-Embodied-AI（私有仓库）
> 目的：多人同时开发不丢代码、不打架

---

## 一、铁律（记住这 3 条）

1. **动手前先 pull，收工前先 push**
2. **每次开工前先 commit**（有历史就不会丢）
3. **永不使用**：`git reset --hard` / `git checkout .` / `git clean -fd`

> 这三条命令会**永久丢弃**未提交的工作，是唯一真正危险的操作。

---

## 二、日常开发流程（标准动作）

### ① 开工：同步最新代码
```bash
cd /path/to/主仓库
git pull --rebase origin main
```

### ② 干活：写代码 / 改文件
（正常编辑即可）

### ③ 中途：随时查看状态
```bash
git status              # 看有哪些改动
git diff                # 看具体改了啥
```

### ④ 收工：提交 + 推送
```bash
git add -A
git commit -m "feat: 完成了什么（用中文写清楚）"
git pull --rebase origin main    # 推送前再同步一次
git push origin main
```

---

## 三、关键问题：pull 会覆盖我的工作吗？

**不会。** Git 会保护未提交的改动。

| 你的状态 | pull 的结果 |
|---|---|
| 已 commit | ✅ 自动合并，双方都保留 |
| 未 commit + 文件不冲突 | ✅ pull 成功，你的改动保留 |
| 未 commit + 文件冲突 | ⚠️ **pull 报错中止，什么都没变**（你的工作安全）|

**报错长这样**：
```
error: Your local changes to the following files would be overwritten by merge:
    xxx/yyy.py
Please commit your changes or stash them before you merge.
```
👉 这时只需先 `git add -A && git commit -m "..."`，再 pull。

---

## 四、遇到冲突怎么办

```
CONFLICT (content): Merge conflict in xxx/yyy.py
```

**步骤**：
1. 打开冲突文件，搜索 `<<<<<<<`、`=======`、`>>>>>>>`
2. 手动决定保留哪部分（或都留），删掉这些标记行
3. 保存后：
```bash
git add xxx/yyy.py
git rebase --continue      # 如果是 rebase
# 或
git commit                 # 如果是 merge
```
4. 实在搞不定，**把报错发群里**，别乱试命令

---

## 五、提交信息规范（方便回溯）

格式：`类型: 简短说明`

| 类型 | 用途 | 例子 |
|---|---|---|
| feat | 新功能 | `feat: 实现 YOLO 目标跟踪` |
| fix | 修 bug | `fix: 修复串口偶发丢包` |
| docs | 文档 | `docs: 更新工作清单` |
| refactor | 重构 | `refactor: 拆分电机控制模块` |
| chore | 杂项 | `chore: 清理编译产物` |

---

## 六、什么该提交、什么不该

| ✅ 应该提交 | ❌ 不要提交 |
|---|---|
| 源码（.v / .py / .c）| 编译产物（.log / .sbit / 位流）|
| 文档（.md）| 大文件（.rar / .zip / 大图）|
| 工程配置、约束文件 | 临时文件、缓存（__pycache__）|
| 测试代码 | 个人环境（venv）|

> 仓库 `.gitignore` 已配好，会自动忽略这些。

---

## 七、分支策略（可选，人多时用）

**小团队直接在主分支 main 上开发即可**（简单）。

如果想更稳妥，可以一人一分支：
```bash
git checkout -b dev-任务5          # 建分支
# ...干活...
git push -u origin dev-任务5        # 推分支
# 然后在 GitHub 上发 Pull Request 合并到 main
```

---

## 八、常见救命操作

### 查看自己改了哪些文件
```bash
git status
```

### 只想暂时放一下改动（不想 commit）
```bash
git stash              # 暂存
git pull --rebase origin main
git stash pop          # 恢复
```

### 忘记刚才干了什么
```bash
git log --oneline -10          # 看提交历史
git reflog                     # 看所有操作记录（能找回误删的提交）
```

### 不小心改了不该改的，想还原单个文件
```bash
git checkout HEAD -- 路径/文件名    # 还原这个文件到上次提交状态
```
> ⚠️ 只对**单个文件**用，且你确定不要这个文件的改动了。

---

## 九、一句话总结

> **先 commit 再 pull，冲突就报错，报错就找人。**
> 只要不碰 `reset --hard` / `checkout .` / `clean -fd`，你的代码就不会丢。

---

## 十、有疑问怎么办

1. 先 `git status` 看清状态
2. 把命令和报错**原样发群里**
3. 队友/Codex 帮你判断，别自己乱试危险命令
