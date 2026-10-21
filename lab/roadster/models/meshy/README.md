# Meshy 车模

`roadster-meshy-clean.glb` 是去墙原白模，汽车20,264点、40,564面原样保留。原始文件与删除墙体另存，未覆盖。

`materials-reviewed/roadster-meshy-colored.glb` 是 Kimi + Codex 着色版。Kimi k3/max 依据官网图提出八类材质，Codex修正边界并烘焙颜色、粗糙度/金属度、发光和清漆图。四张PNG嵌入GLB，也有独立文件。所有原三角面及表面保持；顶点因独立UV小区而重复。

网站的模型对照和三个训练视窗使用这份派生模型。模型原始单位未标定，仅在显示时统一车长4.1m，保留比例。材质不是原厂UV；原有几何小凹凸和简化底部仍保留，不能视为实车精确复原。

调用与读图证据见 `material-provenance.json`。Kimi原始规则在 `materials-kimi/`，Codex修订与几何核对在 `materials-reviewed/`。此前CLI免费套餐拒绝的记录与用户后来手动提供模型的记录分开保存。
