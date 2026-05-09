# Library Documentation Index

> 版本基线：v1.7.0

## 文档列表

| 文档 | 描述 | 必读 |
|------|------|:----:|
| [architecture.md](architecture.md) | 分层架构、数据流、依赖关系（SSOT） | ✅ |
| [development.md](development.md) | 通用开发流程、编译约束、调试经验 | ✅ |
| [protocol-parameters-bilingual.md](protocol-parameters-bilingual.md) | 协议帧格式、参数中英对照、裁剪表 | ✅ |

## 文档边界规则

- `docs/` 下的文档描述**驱动库的通用设计**，不涉及具体硬件平台细节
- 实例级文档（板级管脚、驱动适配说明、实例构建）位于 `examples/<instance>/` 下
- 不要在同一文档中混合库级和实例级内容

## 版本历史

完整版本历史见 [CHANGELOG.md](../CHANGELOG.md)。
