# 2027 RMUL QUST全向轮步兵电控代码
# 2026/10/12
使用同济sp战队开源的电控中间件的底盘代码

# 工具链
gcc+cmake+openocd+cotex-debug+daplink
vscode:
f7快捷编译
ctrl+shift+B快捷烧录

# 代码架构
基于freertos嵌入式实时系统
sp_middleware同济sp战队开源代码中间件
application放置业务层代码逻辑实现（freertosTask等）
core放置cubemx生成的代码，一般不进行修改

# 整车架构
双c板设计，底盘c板+云台c板，，，，，