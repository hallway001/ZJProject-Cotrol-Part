# Git上传说明

## 本地仓库已准备就绪

✅ Git仓库已初始化  
✅ 所有文件已添加并提交  
✅ 包含33个文件，共3861行代码

---

## 连接到GitHub仓库并上传

### 方法1：使用提供的脚本（推荐）

#### Windows用户：
```bash
.git_upload.bat https://github.com/你的用户名/仓库名.git
```

#### Linux/Mac用户：
```bash
chmod +x .git_upload.sh
./git_upload.sh https://github.com/你的用户名/仓库名.git
```

---

### 方法2：手动执行命令

#### 步骤1：添加远程仓库
```bash
# 如果还没有远程仓库，添加它
git remote add origin https://github.com/你的用户名/仓库名.git

# 如果已存在，更新URL
git remote set-url origin https://github.com/你的用户名/仓库名.git
```

#### 步骤2：检查远程仓库状态
```bash
# 查看远程仓库信息
git remote -v

# 获取远程仓库内容
git fetch origin
```

#### 步骤3：强制推送（删除GitHub上的旧代码）

⚠️ **警告**：这将删除GitHub仓库中的所有现有内容，用当前版本替换！

```bash
# 强制推送到main分支
git push -f origin main
```

#### 如果远程仓库是空的（首次推送）：
```bash
# 正常推送
git push -u origin main
```

---

### 方法3：如果已有GitHub仓库需要清空

如果你想完全清空GitHub仓库并上传新代码：

1. **在GitHub网页上删除仓库**（Settings → Delete this repository），然后：
   ```bash
   git remote add origin https://github.com/你的用户名/新仓库名.git
   git push -u origin main
   ```

2. **或者使用强制推送覆盖**：
   ```bash
   git remote add origin https://github.com/你的用户名/仓库名.git
   git push -f origin main
   ```

---

## 验证上传结果

推送完成后，访问你的GitHub仓库页面，应该能看到：
- ✅ 所有源文件（src/, include/）
- ✅ 配置文件（config/）
- ✅ Launch文件（launch/）
- ✅ 工具脚本（scripts/）
- ✅ 文档（README.md, PROJECT_SUMMARY.md）
- ✅ .gitignore文件

---

## 后续更新

以后更新代码时：
```bash
git add .
git commit -m "更新描述"
git push origin main
```

---

## 注意事项

1. **备份**：强制推送前请确保已备份重要数据
2. **分支**：如果GitHub仓库使用`master`分支，请将命令中的`main`改为`master`
3. **权限**：确保你有仓库的写权限
4. **SSH/HTTPS**：可以使用SSH地址（`git@github.com:用户名/仓库名.git`）替代HTTPS

---

## 问题排查

### 如果推送失败：
```bash
# 检查远程仓库URL
git remote -v

# 检查分支名称
git branch

# 检查认证（可能需要配置GitHub凭证）
git config --global user.name "你的用户名"
git config --global user.email "你的邮箱"
```

### 如果需要强制覆盖：
```bash
git push -f origin main
```

---

**完成后，你的GitHub仓库将包含完整的装载车控制模块代码！** 🎉

