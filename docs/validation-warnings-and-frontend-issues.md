# Catálogo de avisos, validações e bloqueios do planejador de trajetórias

**Projeto:** Caesar Compass  
**Escopo:** backend Python e futura camada de apresentação (frontend)  
**Objetivo:** transformar validações atualmente expressas como exceções em mensagens estruturadas, compreensíveis e acionáveis pelo usuário antes da execução do planejamento.

---

## 1. Contexto e comportamento desejado

Atualmente, diferentes módulos validam entradas e resultados por meio de exceções como `ValueError`, `RuntimeError` e `KeyError`. O planejador captura parte dessas exceções para rejeitar candidatos. Se todos os métodos forem rejeitados, a execução termina com erro.

Esse comportamento é adequado para proteger o cálculo, mas não é adequado como experiência de usuário: a aplicação não deve simplesmente parar e apresentar uma exceção técnica quando o problema puder ser identificado antes da execução.

A implementação futura deve separar:

1. **Aviso (warning):** condição que merece atenção, mas não impede necessariamente o cálculo.
2. **Erro de entrada (error):** informação inválida ou ausente que o usuário precisa corrigir.
3. **Bloqueio (blocking):** condição que torna o planejamento matematicamente impossível, inconsistente ou inseguro segundo as restrições configuradas.
4. **Rejeição de candidato (candidate rejection):** um método específico não encontrou solução válida. Isso não deve interromper a comparação enquanto houver outros métodos candidatos.
5. **Erro interno (internal error):** falha inesperada de implementação, que deve ser registrada para diagnóstico e não apresentada como se fosse erro de preenchimento do usuário.

O frontend deverá apresentar os problemas antes de ativar o cálculo sempre que for possível validá-los com os dados de entrada. Validações que dependem do resultado calculado devem ser apresentadas durante a análise dos candidatos ou no resumo final.

## 2. Contrato de dados recomendado

Não utilizar texto de exceção como interface entre backend e frontend. Criar um objeto de validação padronizado.

Exemplo de estrutura:

```json
{
  "code": "INC_ABOVE_CONFIGURED_LIMIT",
  "severity": "warning",
  "blocking": false,
  "category": "trajectory_geometry",
  "field": "constraints.max_inclination",
  "message": "A inclinação excede o limite configurado.",
  "details": {
    "measured": 102.4,
    "limit": 90.0,
    "unit": "deg"
  },
  "suggested_action": "Revise o limite de inclinação ou os parâmetros da trajetória.",
  "method": "slant",
  "station": 12
}
```

### Campos obrigatórios

- `code`: identificador estável, independente do idioma.
- `severity`: `info`, `warning` ou `error`.
- `blocking`: indica se impede a execução.
- `category`: grupo funcional da validação.
- `message`: mensagem curta para o usuário.
- `details`: valores observados, limites e contexto técnico.
- `suggested_action`: orientação objetiva de correção.

### Campos contextuais opcionais

- `field`: campo do formulário relacionado.
- `method`: método que gerou o aviso ou foi rejeitado.
- `station`: índice ou identificação do station.
- `target`: alvo relacionado.
- `candidate_id`: candidato relacionado.
- `source`: módulo ou etapa interna que gerou a validação.

A API deve retornar uma lista de validações. Não interromper a resposta na primeira ocorrência. O frontend deve conseguir agrupar mensagens por etapa, método, alvo e campo.

## 3. Catálogo de validações identificadas no código atual

As mensagens abaixo foram identificadas nos módulos existentes da branch principal. A classificação de severidade é uma proposta funcional; deverá ser confirmada pela equipe de engenharia de trajetórias.

### 3.1 Dados obrigatórios ausentes ou inválidos

| Código sugerido | Condição identificada | Comportamento atual | Tratamento proposto |
|---|---|---|---|
| `MISSING_REQUIRED_FIELD` | Campo obrigatório ausente em `last_data`, `target`, `params` ou parâmetros do método | `KeyError` | Erro bloqueante, indicar o campo |
| `NON_NUMERIC_INPUT` | Valor não numérico, nulo ou não finito | `ValueError` ou falha de conversão | Erro bloqueante |
| `INVALID_MD_ORDER` | MD final menor ou igual ao MD inicial | `ValueError` | Erro bloqueante |
| `MD_NOT_MONOTONIC` | MD não estritamente crescente entre stations | `ValueError` no núcleo Minimum Curvature | Erro bloqueante |
| `INVALID_SURVEY_ARRAYS` | MD, INC e AZI com tamanhos diferentes ou menos de dois stations | `ValueError` | Erro bloqueante |
| `INVALID_ANGLE_VALUE` | INC ou AZI fora da faixa definida para o método ou configuração | Validações distintas por módulo | Erro bloqueante ou aviso conforme a convenção configurada |
| `INVALID_TARGET_COORDINATES` | Coordenadas TVD/NS/EW ausentes ou inválidas | Falha de chave/conversão | Erro bloqueante |

**Requisito de interface:** destacar o campo específico e preservar os demais dados preenchidos. Não apresentar traceback ou nome de exceção ao usuário.

### 3.2 Inclinação (INC), incluindo valores acima de 90°

Há uma divergência de limites entre os módulos atuais:

- `WellTrajectory` aceita valores numéricos de INC sem impor limite explícito de 90° ou 180°.
- `build_segment` valida a INC final entre 0° e 180°.
- `prepare_last_data_at_kop` aceita INC entre 0° e 180°.
- `PlanningConstraints.max_inclination` tem valor padrão de 90°.
- `evaluator.py` e `multi_target.py` rejeitam stations acima do limite configurado.
- `slant.py` define `MAX_INC = 90.0` e limita a INC inicial e a solução a esse valor.
- `optimum_align.py` utiliza limite numérico de inclinação entre 0 e π radianos no solver.

**Conclusão:** INC superior a 90° não deve ser automaticamente classificada como erro global. A convenção de inclinação e o limite operacional precisam ser definidos na configuração do projeto/poço. Entretanto, cada método deve declarar sua faixa suportada.

| Código sugerido | Condição | Severidade proposta | Ação |
|---|---|---|---|
| `INC_ABOVE_CONFIGURED_LIMIT` | INC excede `max_inclination` configurado | Erro bloqueante para aquele planejamento | Ajustar trajetória ou limite autorizado |
| `INC_OUTSIDE_METHOD_RANGE` | INC é válida no modelo geral, mas não suportada pelo método selecionado | Rejeição do candidato; não bloquear outros métodos | Selecionar método compatível ou alterar parâmetros |
| `INC_ABOVE_90_DEG` | INC está entre 90° e 180° | Aviso informativo, se essa faixa for permitida pela configuração | Confirmar convenção e compatibilidade dos métodos |
| `INC_OUT_OF_PHYSICAL_RANGE` | INC fora de 0° a 180° | Erro bloqueante | Corrigir INC |
| `ARRIVAL_INCLINATION_MISMATCH` | INC de chegada não corresponde à orientação exigida para o próximo alvo | Rejeição do candidato | Rever orientação/alinhamento dos alvos |

**Decisão necessária da equipe:** definir se o sistema operará com INC de 0° a 180° em todos os métodos ou se haverá métodos limitados a 90°. Não restringir o modelo global a 90° apenas porque o Slant atual possui essa limitação.

### 3.3 Dogleg Severity (DLS)

Validações encontradas:

- Slant define `MAX_DLS = 30°/30m`.
- DLS fornecido ao Slant deve ser positivo e não superior a 30°/30m.
- Dogleg Toolface rejeita DLS negativo, mas atualmente não aplica limite máximo próprio.
- O evaluator e o planejador multialvo verificam DLS calculado contra `constraints.max_dls`.
- Optimum Align possui rotina que aumenta os DLS de ambos os arcos quando o solver não fecha a geometria. Esse relaxamento pode produzir valores acima do limite operacional se não houver uma verificação final coerente.

| Código sugerido | Condição | Severidade proposta | Ação |
|---|---|---|---|
| `DLS_NEGATIVE` | DLS menor que zero | Erro bloqueante | Informar DLS não negativo |
| `DLS_ZERO_NOT_ALLOWED` | Método exige DLS positivo e recebeu zero | Erro bloqueante para o método | Informar DLS positivo |
| `DLS_ABOVE_METHOD_LIMIT` | DLS excede limite interno do método | Rejeição do candidato | Selecionar outro método ou rever parâmetros |
| `DLS_ABOVE_WELL_LIMIT` | DLS calculado excede limite configurado para o poço | Rejeição do candidato | Ajustar trajetória |
| `DLS_RELAXATION_EXCEEDED_LIMIT` | Optimum Align elevou DLS durante a busca acima do limite do poço | Rejeição do candidato | Não aceitar relaxamento acima do limite |
| `DLS_SPIKE` | DLS pontual muito superior aos stations adjacentes ou ao padrão esperado | Aviso de revisão | Verificar survey, espaçamento e interpolação |

**Requisito:** diferenciar DLS solicitado pelo usuário, DLS utilizado pelo solver e DLS efetivamente calculado na trajetória final. Exibir o maior DLS e o station em que ocorreu.

### 3.4 Alvo e fechamento geométrico

Validações encontradas:

- O alvo precisa conter TVD, NS e EW.
- O Slant rejeita alvo com TVD acima do ponto inicial (diferença de TVD negativa).
- Hold rejeita alvo que não esteja à frente da direção tangente.
- Optimum Align pode não convergir ou não fechar a geometria dentro da tolerância numérica.
- O planejador multialvo compara erro de posição em TVD, NS e EW.
- Dois alvos consecutivos na mesma posição são rejeitados.
- A orientação de chegada a um alvo intermediário deve apontar geometricamente para o próximo alvo.

| Código sugerido | Condição | Severidade proposta | Ação |
|---|---|---|---|
| `TARGET_BEHIND_CURRENT_TVD` | Slant recebe alvo com TVD menor que o ponto inicial | Rejeição do Slant; avaliar outros métodos | Rever alvo ou método |
| `TARGET_BEHIND_HOLD_DIRECTION` | Projeção do alvo sobre a tangente do Hold é nula ou negativa | Rejeição do Hold | Selecionar outro método |
| `TARGETS_COINCIDENT` | Alvo seguinte possui a mesma posição do alvo atual | Erro bloqueante para sequência | Corrigir coordenadas ou ordem |
| `TARGET_NOT_REACHED` | Ponto final não atinge alvo dentro da tolerância | Rejeição do candidato | Rever parâmetros/tolerância |
| `TARGET_CLOSURE_RESIDUAL` | Solver converge numericamente, mas resíduo geométrico permanece acima da tolerância | Rejeição do candidato | Rever geometria e parâmetros |
| `TARGET_ARRIVAL_ALIGNMENT_MISMATCH` | INC/AZI final não aponta para o próximo alvo | Rejeição do candidato | Corrigir orientação ou posição dos alvos |
| `TARGET_ORDER_INFEASIBLE` | Não existe solução viável após um alvo intermediário | Bloqueio do planejamento completo | Rever ordem, espaçamento ou métodos |
| `TARGET_TOLERANCE_EXCEEDED` | Erro máximo de coordenadas supera tolerância configurada | Rejeição do candidato | Rever precisão/tolerância |

**Requisito de interface:** exibir separadamente os erros de TVD, NS e EW, além do erro máximo utilizado na validação. Não resumir tudo em uma mensagem genérica de alvo não atingido.

### 3.5 Optimum Align e Project Back

Validações encontradas:

- Dogleg 1 e Dogleg 2 são obrigatórios quando Optimum Align é executado.
- Project Back deve ser uma estrutura válida com método reconhecido.
- Métodos de Project Back disponíveis no código: extensão, TVD, INC e INC/AZI; há também o caso terminal de superfície.
- DLS zero impede alguns cálculos inversos de extensão.
- Project Back por TVD pode falhar se a componente vertical unitária for nula.
- A rotina de relaxamento de DLS pode aumentar os doglegs durante a busca.
- Para alvo intermediário, orientação explícita conflitante com o próximo alvo é rejeitada.
- Para o último alvo, INC e AZI finais são obrigatórios.

| Código sugerido | Condição | Severidade proposta | Ação |
|---|---|---|---|
| `OPT_ALIGN_DOGLEG_MISSING` | Dogleg 1 ou 2 não informado | Erro bloqueante para Optimum Align | Preencher os dois valores |
| `PROJECT_BACK_INVALID_ITEM` | Item Project Back sem método ou com estrutura inválida | Erro bloqueante | Corrigir configuração |
| `PROJECT_BACK_UNSUPPORTED_METHOD` | Método Project Back não reconhecido | Erro bloqueante para o candidato | Selecionar método disponível |
| `PROJECT_BACK_ZERO_DLS` | Cálculo inverso exige DLS diferente de zero | Erro bloqueante para o candidato | Informar DLS positivo |
| `PROJECT_BACK_VERTICAL_SINGULARITY` | Cálculo por TVD não consegue determinar extensão por componente vertical nula | Rejeição do candidato | Usar outro parâmetro de Project Back |
| `PROJECT_BACK_NEGATIVE_EXTENSION` | Extensão calculada é negativa ou fisicamente incompatível | Rejeição do candidato | Rever os parâmetros |
| `OPT_ALIGN_NO_CONVERGENCE` | Solver não encontrou solução | Rejeição do candidato | Tentar outro método ou configuração |
| `OPT_ALIGN_DLS_RELAXED` | Solver precisou elevar DLS em relação aos valores solicitados | Aviso, desde que continue dentro do limite operacional | Informar DLS solicitado e efetivo |
| `OPT_ALIGN_DLS_LIMIT_EXCEEDED` | DLS relaxado excede limite do poço | Rejeição do candidato | Não aceitar a solução |
| `OPT_ALIGN_FINAL_ORIENTATION_REQUIRED` | Último alvo não possui INC/AZI final | Erro bloqueante | Informar orientação final |
| `OPT_ALIGN_ORIENTATION_CONFLICT` | INC/AZI informada conflita com alinhamento ao próximo alvo | Erro bloqueante para o candidato | Corrigir orientação ou coordenadas |

### 3.6 Slant

Validações encontradas:

- Exatamente dois parâmetros devem ser conhecidos entre KOP, DLS, INC e HOLD_LEN.
- Na otimização externa, HOLD_LEN é fixado em cada tentativa e deve existir exatamente mais um parâmetro conhecido entre KOP, DLS e INC.
- KOP não pode ser anterior ao MD atual.
- INC e DLS devem respeitar os limites internos atuais do Slant.
- Hold length não pode ser negativo.
- O solver deve convergir e fechar a geometria dentro da tolerância.
- O target TVD não pode estar acima do ponto atual.
- O algoritmo atual assume geometria vertical + build + hold, e usa departure horizontal agregado. A compatibilidade com trajetórias que exigem INC acima de 90° ou geometrias não cobertas deve ser explicitada.

| Código sugerido | Condição | Severidade proposta | Ação |
|---|---|---|---|
| `SLANT_PARAMETER_COUNT_INVALID` | Quantidade de parâmetros conhecidos diferente de dois | Erro bloqueante para Slant | Selecionar exatamente dois parâmetros |
| `SLANT_HOLD_PARAMETER_CONFIGURATION_INVALID` | HOLD_LEN fixo sem exatamente mais um parâmetro conhecido | Erro bloqueante para otimização Slant | Ajustar parâmetros conhecidos |
| `SLANT_KOP_BEFORE_CURRENT_MD` | KOP anterior ao MD atual | Erro bloqueante | Corrigir KOP |
| `SLANT_HOLD_LENGTH_NEGATIVE` | HOLD_LEN negativo | Erro bloqueante | Corrigir comprimento |
| `SLANT_INCLINATION_UNSUPPORTED` | INC incompatível com faixa atualmente suportada pelo Slant | Rejeição do candidato | Usar método compatível ou ampliar algoritmo |
| `SLANT_SOLVER_NO_CONVERGENCE` | Solver não convergiu | Rejeição do candidato | Tentar outro conjunto de parâmetros |
| `SLANT_GEOMETRY_NOT_CLOSED` | Resíduo geométrico acima de 1e-3 | Rejeição do candidato | Rever parâmetros |
| `SLANT_NO_VALID_HOLD_LENGTH` | Nenhum HOLD_LEN avaliado gerou candidato válido | Rejeição do Slant | Ampliar intervalo ou usar outro método |

### 3.7 Hold

Validações encontradas:

- O alvo deve conter TVD, NS e EW.
- A projeção do alvo na direção tangente precisa ser positiva.
- A projeção calculada não garante, sozinha, que o ponto final coincida com o alvo; o evaluator deve confirmar o fechamento tridimensional.

| Código sugerido | Condição | Severidade proposta | Ação |
|---|---|---|---|
| `HOLD_TARGET_NOT_AHEAD` | Alvo não está à frente da tangente | Rejeição do Hold | Selecionar outro método |
| `HOLD_TARGET_OFF_AXIS` | Projeção existe, mas o alvo não está sobre a linha de Hold dentro da tolerância | Rejeição do Hold | Selecionar outro método |
| `HOLD_ZERO_EXTENSION` | Extensão projetada nula | Rejeição do Hold | Rever alvo |
| `HOLD_TARGET_NOT_REACHED` | Ponto calculado não fecha o alvo | Rejeição do Hold | Rever geometria |

### 3.8 Nudge

Estado atual: o adaptador implementa apenas o modo `MdIncAzi`. Outros modos ainda não foram portados.

Validações encontradas:

- Modo diferente de MdIncAzi gera `NotImplementedError`.
- Exige MD final, INC final e AZI final.
- MD final precisa ser maior que MD inicial.
- INC final deve estar entre 0° e 180° no construtor de segmento.

| Código sugerido | Condição | Severidade proposta | Ação |
|---|---|---|---|
| `NUDGE_MODE_UNSUPPORTED` | Modo selecionado ainda não foi implementado | Método indisponível; não interromper outros candidatos | Selecionar modo disponível |
| `NUDGE_FINAL_MD_MISSING` | MD final ausente | Erro bloqueante para Nudge | Informar MD |
| `NUDGE_FINAL_INC_MISSING` | INC final ausente | Erro bloqueante para Nudge | Informar INC |
| `NUDGE_FINAL_AZI_MISSING` | AZI final ausente | Erro bloqueante para Nudge | Informar AZI |
| `NUDGE_INVALID_FINAL_MD` | MD final não é maior que MD inicial | Erro bloqueante | Corrigir MD |

**Observação de escopo:** o adaptador atual não equivale a todos os modos do Nudge original. A interface deve apresentar apenas os modos efetivamente suportados.

### 3.9 Dogleg Toolface

Validações encontradas:

- Exige Toolface, DLS e extensão.
- Extensão deve ser positiva.
- DLS não pode ser negativo.
- O código atual não impõe limite superior de DLS no método; a trajetória resultante deve ser avaliada contra o limite global.

| Código sugerido | Condição | Severidade proposta | Ação |
|---|---|---|---|
| `DOGLEG_TF_PARAMETER_MISSING` | Toolface, DLS ou extensão ausente | Erro bloqueante | Preencher campo |
| `DOGLEG_TF_INVALID_EXTENSION` | Extensão menor ou igual a zero | Erro bloqueante | Informar extensão positiva |
| `DOGLEG_TF_NEGATIVE_DLS` | DLS negativo | Erro bloqueante | Informar DLS não negativo |
| `DOGLEG_TF_DLS_LIMIT_EXCEEDED` | DLS calculado acima do limite do poço | Rejeição do candidato | Reduzir DLS ou rever trajetória |
| `DOGLEG_TF_RESULT_OUT_OF_RANGE` | Orientação resultante fora da faixa configurada | Rejeição do candidato | Rever Toolface e parâmetros |

### 3.10 KOP e preparação do last_data

KOP é uma etapa externa de preparação do estado físico inicial. Não deve ser incorporado à função Optimum Align.

Validações encontradas:

- Exige depth, inclination, azimuth, verticalDepth, NS e EW.
- KOP não pode ser anterior ao último MD.
- Dados devem ser numéricos.
- INC atual precisa estar entre 0° e 180°.
- A projeção até o KOP é feita em Hold, mantendo a orientação.

| Código sugerido | Condição | Severidade proposta | Ação |
|---|---|---|---|
| `KOP_BEFORE_LAST_SURVEY` | KOP menor que MD do último survey | Erro bloqueante | Corrigir KOP |
| `KOP_EQUAL_LAST_SURVEY` | KOP igual ao MD atual | Informação: nenhuma extensão será aplicada | Prosseguir sem projeção |
| `KOP_INVALID_INC` | INC fora de 0° a 180° | Erro bloqueante | Corrigir survey |
| `KOP_INVALID_SURVEY_DATA` | Dados obrigatórios ausentes ou inválidos | Erro bloqueante | Corrigir survey |
| `KOP_HOLD_PROJECTION` | Preparação adicionou extensão em Hold até o KOP | Informação | Mostrar MD inicial, KOP e extensão aplicada |

## 4. Validações gerais da trajetória final

Além das validações específicas dos métodos, toda trajetória candidata deve passar por uma validação comum.

| Código sugerido | Verificação | Tratamento |
|---|---|---|
| `PATH_MD_NOT_MONOTONIC` | MD estritamente crescente | Rejeitar candidato |
| `PATH_DISCONTINUITY` | Posição ou orientação inicial do trecho não coincide com o final do trecho anterior | Rejeitar candidato |
| `PATH_DLS_LIMIT_EXCEEDED` | DLS acima do limite configurado | Rejeitar candidato |
| `PATH_INC_LIMIT_EXCEEDED` | INC acima do limite configurado | Rejeitar candidato |
| `PATH_EXTENSION_BELOW_MINIMUM` | Trecho menor que extensão mínima | Rejeitar candidato |
| `PATH_EXTENSION_ABOVE_MAXIMUM` | Trecho maior que extensão máxima | Rejeitar candidato |
| `PATH_DUPLICATE_STATION` | Station duplicado ou extensão nula indevida | Rejeitar candidato ou consolidar conforme regra |
| `PATH_NON_FINITE_RESULT` | Coordenadas, ângulos ou métricas com NaN/Infinity | Rejeitar candidato |
| `PATH_TARGET_ORDER_INVALID` | Alvos atingidos fora da ordem solicitada | Rejeitar planejamento |
| `PATH_ARRIVAL_ALIGNMENT_INVALID` | Tangente no alvo intermediário não aponta para o próximo alvo | Rejeitar candidato |

A verificação de continuidade deve considerar posição e orientação (INC/AZI), não apenas MD. Recomenda-se tolerância numérica explícita para comparação de coordenadas e ângulos.

## 5. Como o frontend deve apresentar as validações

### Antes de executar

Validar o que depende somente das entradas:

- campos obrigatórios;
- tipos, valores finitos e faixas;
- quantidade e combinação de parâmetros;
- ordem e duplicidade de alvos;
- compatibilidade do método com os parâmetros;
- limites de INC, DLS, extensão e KOP;
- orientação final exigida.

O botão de executar deve permanecer desabilitado apenas quando existir validação com `blocking=true`. Warnings não bloqueantes devem ser exibidos para confirmação ou ciência do usuário, conforme a regra de negócio.

### Durante a execução

O backend deve retornar o resultado de cada método, inclusive candidatos rejeitados:

- método avaliado;
- sucesso ou rejeição;
- código e mensagem de cada validação;
- métricas relevantes;
- motivo de rejeição.

A falha de um método não deve encerrar a comparação se outros métodos puderem produzir uma solução válida.

### Após a execução

Apresentar:

- trajetória selecionada;
- métodos utilizados em cada trecho;
- MD final;
- máximo DLS e station correspondente;
- máxima INC e station correspondente;
- erro de fechamento de cada alvo;
- avisos não bloqueantes;
- candidatos rejeitados e motivos resumidos.

Se nenhum candidato for válido, mostrar um resumo consolidado por método e por alvo. Não retornar apenas a última exceção capturada.

## 6. Regras de execução e prioridade

1. Erros de entrada bloqueantes devem ser apresentados antes de iniciar os cálculos.
2. Warnings não bloqueantes não devem interromper o backend.
3. Rejeições devem ser isoladas por método/candidato.
4. A comparação deve continuar enquanto houver métodos a avaliar.
5. O planejamento completo só deve ser declarado inviável quando não existir nenhuma combinação válida de métodos para concluir a sequência de alvos.
6. Falhas inesperadas devem ser registradas separadamente como erro interno.
7. A validação final deve ser independente do método que gerou a trajetória.
8. O limite de INC deve ser configurável e coerente com a faixa suportada por cada método.
9. O relaxamento interno de DLS do Optimum Align nunca pode ultrapassar silenciosamente o limite operacional do poço.
10. O backend deve devolver todos os avisos conhecidos em uma resposta estruturada, sem exigir que o frontend interprete mensagens de exceção.

## 7. Proposta de divisão em issues

### Issue 1 — Criar sistema padronizado de validações
**Objetivo:** implementar o modelo comum de warning/error/blocking/rejection e códigos estáveis.

Critérios de aceite:
- Estrutura de validação serializável em JSON.
- Severidade e propriedade blocking independentes.
- Mensagens amigáveis separadas de detalhes técnicos.
- Nenhuma dependência do frontend em texto de exceção.
- Testes unitários para serialização e agrupamento.

### Issue 2 — Pré-validação dos parâmetros de entrada
**Objetivo:** validar os formulários antes da execução.

Escopo:
- campos obrigatórios e tipos;
- MD, INC, AZI, DLS, KOP e extensão;
- combinações de parâmetros do Slant;
- parâmetros de Optimum Align e Project Back;
- modos suportados de Nudge;
- sequência e coordenadas dos alvos.

Critérios de aceite:
- Todas as validações de entrada retornam códigos estruturados.
- Uma resposta pode conter múltiplas validações.
- O usuário recebe indicação do campo e ação corretiva.
- Nenhum cálculo pesado é iniciado com erro bloqueante.

### Issue 3 — Unificar convenção e limites de inclinação
**Objetivo:** resolver a divergência entre faixa global 0°–180° e limites internos de 90°.

Escopo:
- definir limite global configurável;
- declarar faixa suportada por método;
- revisar Slant, Optimum Align, Hold, Nudge, Dogleg TF, KOP e Minimum Curvature;
- criar casos de teste com INC acima de 90°;
- garantir cálculo coerente de azimute e toolface em trajetórias reversas.

Critérios de aceite:
- INC entre 90° e 180° não é rejeitada apenas por uma regra global implícita.
- Métodos sem suporte à faixa rejeitam somente seus próprios candidatos com código explícito.
- Testes geométricos validam coordenadas e orientação para casos >90°.
- Documentação registra a convenção angular adotada.

### Issue 4 — Padronizar validações de DLS
**Objetivo:** unificar limites de DLS entre métodos e trajetória final.

Escopo:
- validar DLS de entrada e calculado;
- reportar maior DLS e station;
- tratar relaxamento de DLS no Optimum Align;
- diferenciar limite interno do método e limite operacional do poço.

Critérios de aceite:
- Nenhum candidato acima do limite global é aceito.
- Relaxamento de DLS é explicitamente reportado.
- O backend não interrompe a avaliação de outros métodos por rejeição de DLS.

### Issue 5 — Padronizar fechamento de alvo e alinhamento multialvo
**Objetivo:** retornar diagnósticos geométricos detalhados.

Escopo:
- erros independentes de TVD, NS e EW;
- tolerância de posição;
- orientação de chegada INC/AZI;
- alvos coincidentes;
- alvo atrás da tangente de Hold;
- ordem de alvos;
- continuidade entre segmentos.

Critérios de aceite:
- Cada rejeição informa alvo, método, station e erro observado quando aplicável.
- O alinhamento do alvo intermediário é verificado geometricamente.
- O erro de fechamento é retornado em unidades de distância.

### Issue 6 — Isolar falhas por método e consolidar candidatos
**Objetivo:** impedir que a falha de um método interrompa a comparação.

Escopo:
- capturar falhas esperadas como rejeições estruturadas;
- preservar erros inesperados como falhas internas;
- continuar a avaliação dos demais métodos;
- consolidar rejeições por alvo e método;
- retornar inviabilidade somente quando nenhuma combinação completa for possível.

Critérios de aceite:
- Uma falha do Slant não impede testar Hold, Optimum Align ou outros métodos.
- Se houver pelo menos uma combinação válida, o melhor candidato é retornado.
- Se nenhuma combinação for válida, a resposta contém todos os motivos relevantes.

### Issue 7 — Expor diagnósticos de execução para o frontend
**Objetivo:** disponibilizar validações e resultados com contrato estável de API.

Escopo:
- endpoint/retorno de pré-validação;
- retorno de execução com warnings, erros e candidatos rejeitados;
- mensagens e detalhes;
- agrupamento por etapa, método e alvo.

Critérios de aceite:
- Frontend consegue bloquear execução sem interpretar exceções.
- Warnings não bloqueantes podem ser exibidos sem interromper o cálculo.
- Resultado final contém métricas e diagnósticos.
- Contrato documentado e coberto por testes de integração.

### Issue 8 — Auditoria de validações e testes de regressão
**Objetivo:** garantir que todos os métodos e o fluxo multialvo emitam validações consistentes.

Critérios de aceite:
- Testes para todos os códigos documentados.
- Testes para INC >90° e limites configuráveis.
- Testes para DLS acima do limite e relaxamento.
- Testes de alvo não atingido, alvo coincidente e orientação incorreta.
- Testes de falha isolada de método.
- Nenhum aviso esperado depende de traceback para chegar ao usuário.

## 8. Sequência recomendada de implementação

1. Issue 1 — contrato padronizado de validações.
2. Issue 3 — convenção de INC, pois afeta os limites e os métodos.
3. Issues 2 e 4 — pré-validação e DLS.
4. Issue 5 — geometria e multialvos.
5. Issue 6 — isolamento de candidatos.
6. Issue 7 — integração com frontend.
7. Issue 8 — auditoria e regressão.

As issues podem ser desenvolvidas em paralelo após a definição do contrato da Issue 1 e da convenção angular da Issue 3.

## 9. Observações técnicas para a equipe

- Este documento descreve o comportamento encontrado na implementação atual e propõe a forma de expor as validações. Não significa que todos os métodos estejam matematicamente validados contra a implementação original do COMPASS.
- Algumas validações são verificações já presentes no código; outras são requisitos recomendados para fechar lacunas, especialmente continuidade entre segmentos, stations duplicados, spikes de DLS e diagnóstico de erro por coordenada.
- O backend deve continuar sendo a autoridade das validações. O frontend pode antecipar validações simples para melhorar a experiência, mas não substitui a validação do servidor.
- A anticolisão não está incluída no escopo deste catálogo. O contrato de validação deve, entretanto, permitir adicionar futuramente validadores externos de colisão sem alterar a estrutura de resposta.
- Não alterar a matemática dos métodos como parte da simples migração de exceções para warnings. Primeiro padronizar o diagnóstico; mudanças matemáticas devem possuir issues e testes próprios.
