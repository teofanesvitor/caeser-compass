# Caesar Compass — Directional Trajectory Planner

Projeto Python independente para geração, avaliação e comparação de trajetórias direcionais.

## Objetivo

Selecionar a trajetória completa de menor MD final entre as combinações que atingem todos os alvos na ordem definida e respeitam as restrições do projeto.

## Estrutura

- `caesar/core/trajectory.py`: Minimum Curvature e funções geométricas.
- `caesar/planner/kop.py`: preparação do estado físico no KOP.
- `caesar/planner/anti_collision.py`: validação geométrica de afastamento 3D contra múltiplos poços.
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

## Anticolisão

A anticolisão é uma restrição obrigatória para cada candidato quando a configuração
`anti_collision` é fornecida. O planejador avalia a trajetória acumulada contra
todos os poços de referência; uma ramificação que viole o afastamento mínimo é
rejeitada sem eliminar as demais ramificações.

A geometria é amostrada ao longo de cada intervalo de survey. Quando INC/AZI
estão disponíveis em todos os stations, a posição intermediária é integrada
analiticamente pela curva Minimum Curvature e depois discretizada com passo
configurável. O código confere se a posição integrada coincide com as
coordenadas fornecidas no station seguinte, dentro de
`coordinate_tolerance_m`. Doglegs próximos de 180° são rejeitados por
ambiguidade geométrica. Se INC/AZI não estiverem disponíveis, o sistema usa
interpolação linear e identifica explicitamente essa condição no relatório.

O relatório contém a menor aproximação global e uma lista ordenada com a
menor distância, MD planejado e MD de referência para **cada** poço. A
configuração `max_step_m` controla o refinamento espacial (padrão: 2 m) e
`coordinate_tolerance_m` controla a consistência entre coordenadas e
Minimum Curvature (padrão: 1 m). Reduzir o passo melhora a resolução, mas
aumenta o custo computacional. A discretização é uma aproximação numérica:
não substitui uma solução contínua certificada nem uma análise ISCWSA.

Exemplo de configuração no JSON:

```json
{
  "anti_collision": {
    "minimum_clearance_m": 10.0,
    "max_step_m": 2.0,
    "coordinate_tolerance_m": 1.0,
    "coordinate_system": "UTM 24S / SIRGAS2000",
    "vertical_datum": "MSL",
    "reference_wells": [
      {
        "name": "POCO_EXISTENTE_01",
        "coordinate_system": "UTM 24S / SIRGAS2000",
        "vertical_datum": "MSL",
        "survey": [
          {"depth": 1000, "verticalDepth": 980, "ns": 120, "ew": 80},
          {"depth": 1200, "verticalDepth": 1150, "ns": 160, "ew": 110}
        ]
      },
      {
        "name": "POCO_EXISTENTE_02",
        "coordinate_system": "UTM 24S / SIRGAS2000",
        "vertical_datum": "MSL",
        "survey": [
          {"depth": 900, "verticalDepth": 880, "ns": -100, "ew": 40},
          {"depth": 1300, "verticalDepth": 1250, "ns": -60, "ew": 90}
        ]
      }
    ]
  }
}
```

Todos os poços devem usar o mesmo sistema de coordenadas, datum vertical e
unidades (metros). O planejador exige pelo menos um poço de referência quando
a anticolisão está ativada. Os surveys devem conter ao menos dois stations,
com MD estritamente crescente.

Os dados opcionais de incerteza podem ser preservados nos objetos de survey
(por exemplo, campos `uncertainty`, `sigma_*` ou `covariance_*`) ou no
objeto do poço. Nesta etapa, esses dados são apenas reconhecidos e preservados:
**não são propagados nem usados para calcular Separation Factor**. O relatório
indica explicitamente `uncertainty_evaluated: false`. A implementação ISCWSA,
elipsoides/covariâncias, propagação de incerteza e critérios SF serão uma etapa
posterior; não se deve interpretar esta verificação geométrica como avaliação
completa de risco de colisão.

Não existe um limite universal de afastamento: `minimum_clearance_m` deve ser
definido conforme o padrão e os requisitos operacionais aplicáveis. A validação
também exige que o usuário forneça corretamente o sistema de coordenadas e o
datum do poço planejado e dos poços de referência.

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
- `kop_by_method`: KOP absoluto em MD para os métodos indicados. Aceita valor único, lista ou intervalo.
- `max_method_candidates`: limite de segurança para o total de candidatos gerados (padrão: 250).

Tipos de método aceitos no JSON: `optimum_align`, `slant`, `hold`, `nudge`, `dogleg_toolface` e `dogleg_curve_hold`. Os parâmetros de cada método ficam dentro de `params`; consulte o exemplo para a estrutura. Por padrão, `dogleg_curve_hold` é reservado ao alvo final, pois não impõe a tangente necessária para seguir a outro alvo. Os demais métodos são testados nos dois papéis; candidatos incompatíveis são rejeitados pelas verificações geométricas e de restrições. É possível restringir um método com `role`.

### Busca automática de parâmetros

Além de escolher o método, o JSON pode definir variações numéricas nos parâmetros.
Use uma lista para valores discretos ou um objeto `{"min": ..., "max": ..., "step": ...}`
para gerar um intervalo inclusivo. Todos os parâmetros variáveis são combinados
por produto cartesiano. Exemplo:

```json
{
  "dogleg1": {"min": 1.5, "max": 3.0, "step": 0.5},
  "dogleg2": [2.0, 2.5]
}
```

Isso gera seis combinações independentes. A mesma sintaxe pode ser usada em
parâmetros numéricos de Slant, Nudge, Dogleg Toolface e Curve-Hold. Em Slant,
cada valor de `hold_lengths` também vira um candidato independente. O
`kop_by_method` aceita valor único, lista ou intervalo e é aplicado como KOP
físico externo ao método, mantendo a regra de que KOP não pertence ao Optimum
Align. Os candidatos de cada trecho são combinados pelo planejador multialvo,
que seleciona a trajetória completa de menor MD final.

O limite `max_method_candidates` evita explosão combinatória acidental; o
padrão é 250 e a execução falha explicitamente se a configuração ultrapassá-lo.
O limite conta variantes de método/parametrização geradas, não apenas os
candidatos que conseguem atingir o alvo.

O resultado JSON contém a trajetória vencedora em `best.points`, as demais combinações viáveis em `candidates`, os motivos de rejeição em `rejected`, o modo e critério de seleção em `selection` e um resumo com métodos e MD final. A seleção é global: o planejador expande as combinações entre alvos e escolhe a de menor MD final entre as trajetórias completas viáveis. A configuração legada `solvers`/`terminal_solvers` continua aceita, mas `methods` é a interface recomendada. O arquivo de exemplo é ilustrativo e deve
ser ajustado aos dados e parâmetros reais antes de uso operacional.
