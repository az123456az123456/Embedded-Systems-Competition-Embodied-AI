# candidate-26-live-factor：隔离验证

以实际零违例的22为冻结父版本，本轮只改一个组合表达式；功能不通过不运行综合布线，布线重新出现违例则不合入。不给零违例基线叠加未验证改动。

只在nav_live的完整合取式中消去已被fresh、!cpu_trap及!revoked排除的hard_event分量，改用!mot_accept&&!cancel_new。asynchronous_reasons、hard_event本身和所有撤权状态更新不改。

证明范围：全二值条件组合，包括未达状态；不依赖运行状态约束。X/Z传播不作为硬件全二值形式证明范围。

形式证明通过；实际完整owner双实例135例、519224周期逐拍一致。原始机器记录见functional-result.json和tests/results/。输入参考文件只做模块命名隔离，第一次参考模块重名编译错误保留在tests/attempt00-reference-namespace-error/。没有用修改DUT规避失败。

source-frozen-sha256.json、test-evidence-sha256.json和result.json锁定证据；SDC、CST、owner时序状态、motor、CPU协议及其余157个输入文件与22保持一致。构建使用相同高云版本/place2/route2/timing_driven参数；实际时序判据见physical-review.json（构建结束后生成）。
