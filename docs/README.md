# 库文档索引

> 版本基线：v1.9.2
> 项目命名：HYW FOC（中文名：何易位FOC）

## 作用范围

本目录只放置"库级通用文档"：架构、接口契约、开发流程、协议定义。

实例/板级文档放在各实例目录下（例如 `../examples/GD32F303_FOCExplore/`）。

## 文档分层边界

- 放在 `docs/`：可复用的库行为、依赖契约、通用开发规则
- 放在 `examples/<instance>/`：硬件管脚、工具链配置、构建烧录步骤、通道映射
- 库文档可以链接实例文档，但不承载实例专有实现细节

## 本目录文档

| 文档 | 描述 | 必读 |
|------|------|:----:|
| [architecture.md](architecture.md) | 分层架构、数据流、依赖关系（SSOT） | ✅ |
| [development.md](development.md) | 通用开发流程、编译约束、调试经验 | ✅ |
| [protocol-parameters.md](protocol-parameters.md) | 通信协议帧格式、参数表、裁剪规则 | ✅ |

说明：结构类重复文档已移除，不再保留兼容跳转页。

## 当前基线

- 版本基线：v1.9.2
- 项目名称：HYW FOC（何易位FOC）
- 任务目标：下一目标版本见 [NEXT_MISSION.md](../NEXT_MISSION.md)
- 第三方许可证声明：见 [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)

## 相关实例文档（GD32F303）

- [../examples/GD32F303_FOCExplore/README.md](../examples/GD32F303_FOCExplore/README.md)
- [../examples/GD32F303_FOCExplore/DEVELOPMENT.md](../examples/GD32F303_FOCExplore/DEVELOPMENT.md)
- [../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md](../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md)
- [../examples/GD32F303_FOCExplore/hardware/hardware.md](../examples/GD32F303_FOCExplore/hardware/hardware.md)

## 推荐阅读顺序

1. [architecture.md](architecture.md)
2. [development.md](development.md)
3. [protocol-parameters.md](protocol-parameters.md)

## 维护约定

- API、配置头、层级归属变化时，需在同次迭代同步更新本目录文档。
- [CHANGELOG.md](../CHANGELOG.md) 的版本基线需与本索引一致。
- [NEXT_MISSION.md](../NEXT_MISSION.md) 需与当前阶段状态一致（已完成项不可继续标注为进行中）。