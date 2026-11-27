# Caesar Compass — Directional Trajectory Planner

Projeto Python independente para geração, avaliação e comparação de trajetórias direcionais.

## Objetivo
Selecionar a trajetória de menor MD final entre os candidatos que atingem o alvo e respeitam as restrições do projeto.

## Estrutura
- `caesar/core/trajectory.py`: cálculo Minimum Curvature.
- `caesar/planner/`: geração, validação e seleção de candidatos.
- `caesar/methods/`: métodos construtivos (Optimum Align e Slant).
- `tests/`: testes automatizados.

## Instalação
```bash
python -m pip install -e ".[dev]"
pytest
```

## Convenções
Azimute em graus: 0° = Norte, sentido horário. Coordenadas: NS positivo para Norte; EW positivo para Leste. MD/TVD/coordenadas em metros; DLS em °/30 m.

A primeira versão não aplica anticolisão. A arquitetura de avaliação permite adicionar validadores adicionais posteriormente.
