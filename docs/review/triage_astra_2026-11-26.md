# External review #2b (Astra, gpt-6.2-astra, 2026-11-26): triage

Full text: [astra_review_2026-11-26.md](astra_review_2026-11-26.md) (80k prompt tokens, 12k out). **V** = verified in the repo, **P** = plausible, not yet checked.

**Verdict of the review:** not yet for the full M1 kit; buy samples + one GIM for a restrained single-axis rig first.

| # | Finding | Sev. | Triage |
|---|---|---|---|
| 1 | Split big pulleys can't go on: the Ø40 round tube can't pass a 34.4 AF hex / 36.4 AF double-D bore. R14 checked bearings, not the pulley | critical | **V** (loop-40 error). Fix: Ø40 round bore + key/cross-pins or a clamp hub; add "shaft through pulley" to R14 |
| 2 | Shaft → link drive missing: clevis Ø40.4 holes and forearm Ø42 bore have no clamp/key; yaw flange bolt pattern not cut; CF tubes start exactly where their sockets end (zero bond length) | critical | P (geometry matches the description) |
| 3 | D13 stack: shaft only 2 mm into a 12 mm clamp; kit says 70 mm blanks vs ≈ 42 mm modelled | major | P |
| 4 | D16 wrist skirt traps both bearings (seat between smaller bores); spindle not retained downward; dog ±2.5° lash | critical | **V** |
| 5 | D15 pin relief (2.1 mm deep) cuts away the ball capture (equator at 1.0 mm) | critical | **V** |
| 6 | KX MC is an assumed preload, not a qualified clutch; coupon C1 creep hold weaker than review #1 asked (PETG 50 °C/72 h vs 65 °C/168 h, ASA now) | major | P, fair |
| 7 | Wrist pitch KX56 can see ≈ 1.5 N·m bending when the arm rolls (payload along the pitch axis) ≈ lift-off | major | P: my 40 mm-offset argument missed that pose |
| 8 | Balancer: at the 65 mm slot end the die spring is at 41 % (limit 40 %); **pin-B sign** in shoulder.py may be opposite to balancer.py (20 vs 80 mm span) | critical | P: check the sign first |
| 9 | Motors: rated/sample-worst 1.17–1.36 (not ≥ 1.5); masses() still use old parts | major | fair: p95 is the rule we chose; worst-case holds need derating |
| 10 | Thermal corners mixed: derated worst hold → pitch ≈ 117 °C by our own model | major | P |
| 11 | Belts: roll sustained 1.44× (readiness said ≥ 1.5); tension window unspecified; yaw 315 vs 313 uses the adjustment | major | **V** (1.44×) |
| 12 | Accuracy: gravity *model* must correct displacement; stiffness sums magnitudes, and ÷1.5 on cases with peak belt pull | major | fair |
| 13 | P2 not converged at h 4.5 on 1.2 mm skins; P2 elbow max 8.7 MPa → 1.38× | major | fair |
| 14 | R14/R15 nominal only; yaw block fused although "bottom plate bolts on"; belts not in the roll module | major | **V** (fused plate, no belt solid) |
| 15 | Clock-spring capacity formula ignores ribbon thickness: yaw ≈ 306°, not 446° | major | P: likely right; needs the packing model |
| 16 | Power: 15 A PSU vs 33 A peaks; 3 A buck vs 5.4 A for 2 stalled STS; 20 AWG drop | critical | P, fair |
| 17 | Regen clamp: 70 W in a 50 W resistor; no energy spec | critical | **V** (arithmetic) |
| 18 | Cat-0 drops the arm; pre-charge resistor wired permanently in parallel; watchdog can't meet 50 ms | critical | **V** (diagram) |
| 19 | CAN node port map, termination, TTL adapter half-duplex | major | fair |
| 20 | safety.py clamps feed-forward only; kp·e can still command full current; NaN, stale feedback, unlatched faults | critical | **V** |
| 21 | BC dry run 2/20 is a pipeline test only; R16 cross-body 53.8 % (RND_PLAN said 57 %) | major | **V** (stale 57 %) |
| 22 | No commissioning BOM (stand, catch, gauges, thermistors, scope access); coupons C5–C8 stale | major | fair |
