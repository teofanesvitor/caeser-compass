# Caesar Compass — Directional Trajectory Planner

Projeto Python independente para geração, avaliação e comparação de trajetórias direcionais.

## Objetivo

Selecionar a trajetória completa de menor MD final entre as combinações que atingem todos os alvos na ordem definida e respeitam as restrições do projeto.

## Estrutura

- `caesar/core/trajectory.py`: Minimum Curvature e funções geométricas.
- `caesar/planner/kop.py`: preparação do estado físico no KOP.
- `caesar/planner/multi_target.py`: planejamento sequencial multialvo e alinhamento geométrico.
- `caesar/planner/`: geração, validação e seleção de candidatos.
- `caesar/methods/optimum_align.py`: Optimum Align Curve–Hold–Curve e adaptador multialvo.
- `caesar/methods/slant.py`: Slant e busca do comprimento do primeiro Hold.
- `tests/`: testes automatizados.

## Regras do planejamento multialvo

1. Os alvos são interceptados na ordem recebida.
2. O trecho seguinte começa no estado físico completo do trecho anterior.
3. Ao atingir um alvo intermediário, a tangente (INC/AZI) deve apontar geometricamente para o próximo alvo.
4. O Optimum Align recebe a posição do alvo atual e a orientação calculada em direção ao próximo alvo.
5. O KOP não é parâmetro do Optimum Align. O estado deve ser projetado até o KOP antes da chamada, usando `prepare_last_data_at_kop`.
6. A otimização compara combinações completas de métodos e seleciona o menor MD final.
7. O último alvo exige INC/AZI explícitos quando usado pelo adaptador Optimum Align.

A tolerância angular de alinhamento padrão é 0,1°. Pode ser ajustada no `MultiTargetPlanner`. Restrições de DLS e inclinação são avaliadas sobre a trajetória acumulada.

## Slant

O Slant original exige exatamente dois parâmetros conhecidos entre KOP, DLS, INC e HOLD_LEN. A busca de Hold fixa HOLD_LEN em cada candidato e exige mais um parâmetro conhecido. Cada resultado é avaliado pelas restrições do projeto.

## Instalação

```bash
python -m pip install -e ".[dev]"
pytest
```

## Convenções

Azimute em graus: 0° = Norte, sentido horário. Coordenadas: NS positivo para Norte; EW positivo para Leste. MD/TVD/coordenadas em metros; DLS em °/30 m.

A anticolisão ainda não está implementada. A arquitetura de avaliação permite adicionar validadores adicionais posteriormente.
