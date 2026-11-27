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
- `caesar/methods/hold.py`: trecho Hold com orientação constante.
- `caesar/methods/nudge.py`: adaptador Nudge para `MdIncAzi`.
- `caesar/methods/dogleg_tf.py`: Dogleg Toolface explícito e solver Curve–Hold para interceptação de um alvo usando DLS como parâmetro principal.
- `tests/`: testes automatizados.

## Regras do planejamento multialvo

1. Os alvos são interceptados na ordem recebida.
2. O trecho seguinte começa no estado físico completo do trecho anterior.
3. Ao atingir um alvo intermediário, a tangente (INC/AZI) deve apontar geometricamente para o próximo alvo.
4. O Optimum Align recebe a posição do alvo atual e a orientação calculada em direção ao próximo alvo.
5. O KOP não é parâmetro do Optimum Align. Configure `kop_by_method={"Optimum Align": kop_md}` em `MultiTargetPlanner.plan`; o planejador projeta o estado até o KOP, registra o Hold na trajetória e chama o solver com o estado preparado. Métodos omitidos iniciam no survey original.
6. A otimização compara combinações completas de métodos e seleciona o menor MD final.
7. O último alvo pode usar uma estratégia terminal independente por meio de `terminal_solvers`; o Dogleg Toolface Curve–Hold otimiza a orientação final e o comprimento do Hold a partir do DLS informado.
8. A continuidade entre segmentos é verificada em MD, TVD, NS, EW, INC e AZI.

A tolerância angular de alinhamento padrão é 0,1°. Pode ser ajustada no `MultiTargetPlanner`. Restrições de DLS e inclinação são avaliadas sobre a trajetória acumulada.

## Slant

O Slant original exige exatamente dois parâmetros conhecidos entre KOP, DLS, INC e HOLD_LEN. A busca de Hold fixa HOLD_LEN em cada candidato e exige mais um parâmetro conhecido. Cada resultado é avaliado pelas restrições do projeto.

## Métodos portados e pontos de substituição

Os métodos Hold, Nudge (`MdIncAzi`) e Dogleg Toolface usam a mesma interface de pontos e o motor Minimum Curvature do pacote. O Dogleg Toolface possui também `solve_curve_hold(last_data, target, dls=...)`, que procura uma curva de DLS constante seguida de Hold até um único alvo. No multialvo, passe esse método em `terminal_solvers` para reservá-lo ao último alvo. O Nudge está deliberadamente limitado ao modo `MdIncAzi`; os demais modos ainda serão portados. Os módulos de método ficam isolados para permitir substituir as rotinas de cálculo sem alterar o planejador multialvo.

Os adaptadores de trecho retornam `[ponto_inicial, ponto_final]`, permitindo encadeamento sem perda do estado físico. O planejador multialvo recebe os solvers pelo contrato `solve(last_data, target, alignment_target)`; métodos que exigem parâmetros específicos devem ser envolvidos por uma função adaptadora.


### Conectando os métodos ao planejador

Os adapters abaixo apenas fixam os parâmetros de cada método. As funções de cálculo permanecem inalteradas:

```python
from caesar.planner import (
    MultiTargetPlanner, PlanningConstraints,
    optimum_align_solver, slant_solver, hold_solver,
    dogleg_curve_hold_solver,
)

constraints = PlanningConstraints(max_dls=3.0, max_inclination=90.0)
planner = MultiTargetPlanner(constraints)

result = planner.plan(
    last_data=last_data,
    targets=targets,
    solvers={
        "Optimum Align": optimum_align_solver(dogleg1=2.0, dogleg2=2.0),
        "Slant": slant_solver(
            method_flags={
                "kop": True, "dls": False,
                "inclination": False, "hold_len": False,
            },
            values={"kop": 300.0},
            hold_lengths=[50.0, 75.0, 100.0],
            constraints=constraints,
        ),
        "Hold": hold_solver(),
    },
    terminal_solvers={
        "Dogleg Toolface Curve-Hold": dogleg_curve_hold_solver(dls=2.5),
    },
    kop_by_method={"Optimum Align": 1500.0},
)

best_trajectory = result["best"].points

## Instalação

```bash
python -m pip install -e ".[dev]"
pytest
```

## Convenções

Azimute em graus: 0° = Norte, sentido horário. Coordenadas: NS positivo para Norte; EW positivo para Leste. MD/TVD/coordenadas em metros; DLS em °/30 m.

A anticolisão ainda não está implementada. A arquitetura de avaliação permite adicionar validadores adicionais posteriormente.

## Executar por JSON

O planejador pode ser executado sem escrever código Python para cada poço. Copie
`examples/multitarget_example.json`, substitua os dados de exemplo e execute:

```bash
python -m pip install -e ".[dev]"
python -m caesar examples/multitarget_example.json -o resultado.json
```

O arquivo de entrada contém:

- `last_data`: último station do survey, com MD, INC, AZI, TVD, NS e EW.
- `targets`: lista ordenada de alvos. Para alvos intermediários, o programa
  calcula a orientação de chegada apontando para o alvo seguinte.
- `constraints`: limites de DLS, inclinação e tolerância posicional.
- `methods`: conjunto único de métodos candidatos. O planejador os distribui automaticamente entre trechos intermediários e alvo final. O campo opcional `role` aceita `auto` (padrão), `both`, `intermediate` ou `terminal`.
- `kop_by_method`: KOP absoluto em MD para os métodos indicados.

Tipos de método aceitos no JSON: `optimum_align`, `slant`, `hold`, `nudge`, `dogleg_toolface` e `dogleg_curve_hold`. Os parâmetros de cada método ficam dentro de `params`; consulte o exemplo para a estrutura. Por padrão, `dogleg_curve_hold` é reservado ao alvo final, pois não impõe a tangente necessária para seguir a outro alvo. Os demais métodos são testados nos dois papéis; candidatos incompatíveis são rejeitados pelas verificações geométricas e de restrições. É possível restringir um método com `role`.

No Slant, cada valor de `hold_lengths` é transformado em um candidato
independente. Assim, o planejador mantém alternativas para comparar o MD
final da sequência completa, em vez de escolher apenas o melhor Slant local.

O resultado JSON contém a trajetória vencedora em `best.points`, as demais combinações viáveis em `candidates`, os motivos de rejeição em `rejected`, o modo e critério de seleção em `selection` e um resumo com métodos e MD final. A seleção é global: o planejador expande as combinações entre alvos e escolhe a de menor MD final entre as trajetórias completas viáveis. A configuração legada `solvers`/`terminal_solvers` continua aceita, mas `methods` é a interface recomendada. O arquivo de exemplo é ilustrativo e deve
ser ajustado aos dados e parâmetros reais antes de uso operacional.
