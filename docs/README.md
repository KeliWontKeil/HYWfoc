# 文档索引（库级）

## 作用范围

本目录只放“库级通用文档”：架构、接口契约、通用流程、协议定义。

实例/板级文档必须放在各实例目录下（例如 [../examples/GD32F303_FOCExplore/](../examples/GD32F303_FOCExplore/)）。

## 文档分层边界

- 放在 `docs/`：可复用的库行为、依赖契约、通用开发规则。
- 放在 `examples/<instance>/`：硬件管脚、工具链配置、构建烧录步骤、通道映射。
- 库文档可以链接实例文档，但不承载实例专有实现细节。

## 当前基线

- 版本基线：v1.0.0
- 项目命名：HYW FOC（中文名：何易位FOC）
- 任务目标：下一目标版本见 [../NEXT_MISSION.md](../NEXT_MISSION.md)
- 第三方许可证声明：见 [../THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)

## 本目录文档

- [architecture.md](architecture.md)：分层模型、时序架构、数据流、模块依赖约束。
- [development.md](development.md)：通用开发流程、编码标准、验证要求。
- [structure-and-dependency-tree.md](structure-and-dependency-tree.md)：当前文件树与分层依赖快照。
- [protocol-parameters-bilingual.md](protocol-parameters-bilingual.md)：通信协议与运行参数（中英对照）。
- [api-unused-interface-evaluation.md](api-unused-interface-evaluation.md)：未使用接口分类与保留/移除依据。
- [library-structuring-p1.md](library-structuring-p1.md)：P1 结构化迁移说明与文件映射。

## 相关实例文档（GD32F303）

- [../examples/GD32F303_FOCExplore/README.md](../examples/GD32F303_FOCExplore/README.md)
- [../examples/GD32F303_FOCExplore/DEVELOPMENT.md](../examples/GD32F303_FOCExplore/DEVELOPMENT.md)
- [../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md](../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md)
- [../examples/GD32F303_FOCExplore/hardware/hardware.md](../examples/GD32F303_FOCExplore/hardware/hardware.md)

## 推荐阅读顺序

1. [architecture.md](architecture.md)
2. [development.md](development.md)
3. [structure-and-dependency-tree.md](structure-and-dependency-tree.md)
4. [protocol-parameters-bilingual.md](protocol-parameters-bilingual.md)

## 维护约定

- API、配置头、层级归属变化时，需在同次迭代同步更新本目录文档。
- [../CHANGELOG.md](../CHANGELOG.md) 的版本基线需与本索引一致。
- [../NEXT_MISSION.md](../NEXT_MISSION.md) 的目标版本应始终领先当前稳定基线一步。
