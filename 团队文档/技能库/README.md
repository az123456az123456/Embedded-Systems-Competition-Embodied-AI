# 🧰 团队技能库（Skills）

> 这里存放我们开发中「踩过的坑 + 验证过的解法」，做成 Codex 标准 Skill 格式。
> 队友把需要的技能**拷进自己的 Codex** 后，直接对 Codex 说需求就能自动套用，省去重复摸索。

## 有什么技能

| 技能 | 解决什么问题 | 适用场景 |
|---|---|---|
| `python-venv-nosudo` | 无 sudo/无 pip 装 Python 视觉环境 | 受限 Linux、新电脑装 ultralytics/torch |
| `verilog-sim-nosudo` | 无 sudo 装 iverilog/GTKWave 仿真 | 学 Verilog、RTL 仿真 |
| `github-private-auth` | 私有仓库登录/克隆/推送 | 队友拉取比赛仓库 |
| `gowin-eda-setup` | 高云 EDA 下载安装 + License | FPGA 综合/烧录 |
| `camera-device-in-sandbox` | 沙箱看不到摄像头的问题与正确测试法 | 摄像头/视觉调试 |

## 怎么用（给队友）

### 方法 A：装进自己的 Codex（推荐）
把技能文件夹复制到 Codex 技能目录：
```bash
# 把需要的技能拷到自己的 Codex skills 目录
mkdir -p ~/.codex/skills
cp -r 技能库/python-venv-nosudo ~/.codex/skills/
cp -r 技能库/verilog-sim-nosudo ~/.codex/skills/
cp -r 技能库/github-private-auth ~/.codex/skills/
cp -r 技能库/gowin-eda-setup ~/.codex/skills/
cp -r 技能库/camera-device-in-sandbox ~/.codex/skills/
```
然后对 Codex 说「用 python-venv-nosudo 帮我装环境」或直接描述需求（描述匹配时 Codex 会自动选用）。

### 方法 B：当普通文档看
直接打开各技能目录下的 `SKILL.md`，里面有完整步骤和解法，照着做即可。

## 贡献新技能（规范）
1. 在 `技能库/` 下新建文件夹，命名用英文小写 + 连字符（如 `python-venv-nosudo`）
2. 里面必须有一个 `SKILL.md`，开头是 YAML 头（`name` + `description`），正文写步骤
3. 需要自动化的部分放 `scripts/`，详细资料放 `references/`
4. 写好后在群里说一声，让大家同步

## 说明
- 技能内容来自 2026-09-11 团队与 Codex 的实际协作，均已验证 ✅
- 每个技能应保持「小而专」，一个技能只解决一类问题
