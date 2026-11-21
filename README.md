# Bancada de simulação · Seguidor de linha

Protótipo local em Python para testar o fluxo de mapeamento, ajuste iterativo e tomada de tempo. Requer Python 3.10+ com Tkinter, incluído na instalação padrão do Python para Windows. Usa apenas a biblioteca padrão.

## Rodar no Windows

Abra o PowerShell na pasta do projeto e execute:

```powershell
py -3.13 app.py
```

Use a versão instalada no seu computador se for diferente de 3.13.

1. O app abre com a **pista Pro regularizada** (47,35 m). O botão **Exemplo** recarrega essa pista. Para criar outra, clique em **Desenhar**. Use a roda do mouse sobre a pista ou os botões **− / +** para mudar o zoom, e arraste com o botão direito ou do meio para deslocar a vista. **Ajustar vista** reenquadra a pista ou a parte já desenhada. Trace com o botão esquerdo. Você pode soltar para ajustar a vista e retomar pela ponta azul. Solte perto da largada para fechar. Prefira curvas contínuas e evite cruzamentos. O primeiro ponto é a largada e a ordem dos pontos define o sentido. **Salvar pista** e **Abrir pista** usam JSON com a lista `points_m`.
2. Clique em **Mapear**. O robô segue a linha devagar e registra encoders e IMU a 100 Hz. Clique em **Gerar mapa** após completar a volta.
3. Examine a reconstrução na aba **Mapa estimado**. A ferramenta combina giro dos encoders com o giroscópio e marca setores de comprimento variável com notas de rally. **1** é a curva mais fechada e **5** a mais aberta. **D** é direita, **E** é esquerda e **Reta** é o setor quase sem curvatura. A curvatura mais alta de cada setor entra na estimativa de velocidade e Kp.
4. Clique em **Validar** e **Diagnosticar**. Em **Curvas e saídas**, cada curva tem a mesma numeração da pista. A tabela mostra sua parcial, a diferença para a referência, a velocidade de saída medida nos encoders e o limitante provável. Selecione uma região para ver as leituras que sustentam a hipótese. A linha com ▶ identifica o próximo teste. Use **Aplicar proposta** e **Validar** para medi-lo. A aba **Setores técnicos** detalha erro RMS, borda do QTR, saturação de motor e saldo frente ao tempo estimado pelo planejador.
5. Clique em **Tomar tempo** após uma volta de validação completa e com uma pista que passe na checagem geométrica da aba **RoboCore Pro**. A pista padrão passa nessa checagem. Na mesma aba, **Exemplo Pro** carrega um circuito curto para testes rápidos. A tomada usa o melhor perfil já validado, com uma semente de ruído própria por tentativa. A telemetria entre os portões também pode alimentar **Diagnosticar**.

## Regulamento RoboCore Pro

O modo de tomada de tempo segue a revisão do regulamento fornecido de 03/12/2021.

| Seção | Regra aplicada no simulador |
| --- | --- |
| 3.1–3.2 e 5.8–5.10 | Robô autônomo com um perfil fixo. Os comandos de edição e ajuste ficam bloqueados durante a reprodução da tentativa. O gabarito é de 250 × 250 × 200 mm, e as dimensões estimadas do modelo são configuradas em `Robot`. |
| 4.1–4.7 e 4.11–4.15 | Faixa branca nominal de 19 mm e até 60 m. A checagem geométrica estima raio mínimo de 100 mm, separação de 200 mm, cruzamentos de 90° ±5° com 250 mm de reta de cada lado e reta de 1,50 m em torno da zona de partida e chegada. |
| 4.7–4.12 e 4.14 | A chegada fica 1 m antes da largada. A vista desenha a área de 1,00 × 0,40 m, as marcações de largada e chegada à direita e marcações estimadas de mudança de curvatura à esquerda. A faixa não local é verificada perto dos portões. |
| 5.3, 5.5–5.7, 5.11–5.12 | Até três tentativas em cinco minutos. O robô parte parado na área, deve manter o corpo sobre a faixa e parar completamente dentro da mesma área por pelo menos dois segundos. A volta é cronometrada entre as detecções simuladas nos portões. A tabela considera o menor tempo entre voltas válidas. |

A zona é posicionada em relação ao primeiro ponto do arquivo, que marca a largada. Ao desenhar uma pista para a prova, coloque esse ponto em uma reta com espaço para a chegada e para as extensões retas de 25 cm. O cronômetro da tomada começa ao iniciar a primeira tentativa no aplicativo, que representa a entrada do robô na área de prova. Os portões simulados detectam a passagem do centro do robô. Ao selecionar a melhor validação, o simulador cruza a posição na pista com os encoders para calibrar o ponto de parada no centro da zona. A tentativa Pro executa a parada com esse alvo de encoder. A tolerância para representar repouso é de 5 mm/s por roda. O alvo, a posição final em relação à chegada e os tempos dos portões e de repouso ficam no JSON exportável.

A checagem é uma estimativa a partir da polilinha. A vistoria física inclui os vãos dos portões (400 × 200 mm), manta emborrachada preta, contraste, emendas de até ±2 mm e rampas de até 5°. O simulador representa piso plano e as marcas laterais são visuais. A autorização de nova chance por falha de sensor, as decisões dos juízes e as regras de equipe e manutenção são procedimentos de evento. O botão **Nova tomada de tempo** prepara um novo bloco de tentativas.

Ao iniciar qualquer um dos três testes, a aba **Pista e trajetória** anima a volta na velocidade simulada. A placa vermelha aponta para a frente do robô, o rastro azul mostra o caminho percorrido e os pontos do array ficam mais claros conforme o sinal de linha aumenta. O quadro **Detalhe do robô** amplia o modelo e as leituras durante a volta. O cronômetro acompanha o percurso e o resultado aparece ao final.

Use **Animação 1×, 2×, 4× ou 8×** para mudar a reprodução do mapeamento ou da validação, inclusive com o robô em movimento. A telemetria e os tempos de volta usam o tempo simulado. Durante a tomada RoboCore Pro, a reprodução é de 1×.

O contador no topo de **Pista e trajetória** mostra quantas curvas o robô já iniciou e o total do circuito. Durante o percurso, indica reta, curva à direita ou curva à esquerda. A contagem usa a geometria da pista e a posição simulada, de modo que uma curva contínua conta uma vez mesmo que tenha várias notas ou setores de controle.

## Configuração do robô e do perfil

- A aba **Robô** permite editar o diâmetro das rodas, a rotação máxima de saída dos motores, a geometria, os encoders, o QTR, a IMU, os limites de controle e os ganhos do mapeamento. O formulário aceita vírgula decimal. **Abrir configuração** e **Salvar configuração** trabalham com `robo.json`. **Aplicar ao simulador** usa os valores do formulário. Uma mudança inicia uma sessão nesta mesma pista, preserva os resultados anteriores no disco e pede um novo mapeamento.
- Em **Robô → Aderência**, marque **Sistema de sucção** e informe a força descendente em newtons e a massa do robô em gramas. O limite lateral base é o valor com o sistema desligado. Quando ligado, o limite efetivo é `limite_base × (1 + força / (massa × 9,81))`, usando massa em kg. O modelo assume sucção constante e o mesmo coeficiente de atrito. O limite efetivo entra no perfil inicial de curva, na dinâmica, no círculo de tração e no diagnóstico.
- A aba **Parâmetros** permite editar velocidade, Kp, Kd e antecipação de giro em cada trecho. A antecipação vai de 0 a 1 e usa a velocidade dos encoders e a curvatura adiante no mapa para comandar parte do giro antes da correção do QTR. Em perfis antigos o ganho começa em zero. Clique em **Guardar trecho**, depois em **Preparar validação**. A próxima volta compara a proposta manual com a referência, quando houver. O perfil candidato fica registrado no ZIP da sessão.
- Para testar um ajuste em todas as curvas, use **Guardar nas curvas**. Informe a antecipação de giro, a variação percentual de velocidade, ou ambas. O percentual usa a velocidade nominal de cada setor de curva do perfil em uso. Campos vazios preservam o valor do rascunho. Para reproduzir o ensaio da sessão 045, valide primeiro a referência, depois teste antecipação de **0,80**. Se for aceita, teste **+20%** de velocidade nas curvas com o mesmo ganho. Cada proposta segue por **Preparar validação → Validar**.

O mesmo quadro já mostra o robô parado na largada, com velocidade e demanda de aderência zeradas. O **Círculo de tração** mostra a demanda lateral no eixo horizontal e aceleração ou frenagem no vertical. Ambos os eixos usam o limite lateral efetivo, incluindo o ganho de sucção quando ativado. A distância ao centro mede a demanda combinada. A borda representa 100%, a cor fica amarela a partir de 85% e vermelha acima de 100%. Pontos muito fora da borda são comprimidos no desenho para manter a direção visível. O número continua mostrando a porcentagem calculada.

O círculo serve ao diagnóstico. A física do simulador limita o giro pela demanda lateral, e o rastro fica vermelho nessas situações. O QTR continua detectando a perda de linha separadamente.

Na validação e na tomada de tempo, a pista é colorida por grupos de parâmetros e o trecho selecionado pelo controlador ganha um contorno claro. O painel mostra o número do trecho, o grupo, a velocidade-alvo aplicada e os ganhos Kp e Kd. A aba **Trechos e pesos** agrupa trechos com os mesmos valores exibidos de velocidade, Kp, Kd e antecipação. As cores são projetadas proporcionalmente ao comprimento da pista desenhada. A seleção do trecho ativo usa a distância dos encoders.

O mapa calcula a curvatura em amostras internas de aproximadamente 5 cm e atribui graus de pertinência a classes vizinhas. A nota só muda quando a nova classe tem margem suficiente sobre a anterior na mesma direção. Amostras consecutivas com a mesma nota formam um setor. As pertinências médias ficam no campo `membership` do JSON do perfil. O diagnóstico usa o comprimento real de cada setor para calcular o tempo esperado.

Na aba **Mapa estimado**, a vista padrão agrupa notas de até 15 cm com a nota mais próxima e sustentada da mesma curva. Retas e mudanças de lado delimitam os grupos. O contorno reconstruído e os setores usados pelo controle e pelo diagnóstico são preservados. Ative **Mostrar setores técnicos** para ver cada nota original.

As notas de rally e os ganhos do PD continuam ligados aos setores. Durante validações e tomadas de tempo, o planejador começa a reduzir o alvo antes de setores mais lentos, considerando o atraso do motor. O alvo também sobe gradualmente ao sair da curva ou ao recuperar a linha. Os limites iniciais são `acceleration_limit = 1.2` m/s² para a subida do alvo e `braking_limit = 0.65` m/s² para o planejamento da desaceleração. São parâmetros do modelo, definidos em `Robot`, que precisam ser calibrados para representar o robô físico.

Em retas e trechos de baixa curvatura, o controle reduz gradualmente o ganho proporcional de direção (`Kp`) quando a velocidade estimada pelos encoders passa de `straight_kp_speed` (0,85 m/s por padrão). O ganho é suavizado no tempo e nunca cai abaixo de 35% do nominal. Nas curvas mais fechadas, ele volta ao ganho do perfil. O painel lateral mostra o `Kp` aplicado. Use `null` no campo `straight_kp_speed` para reproduzir o controle anterior sem essa adaptação.

Cada sessão grava pista, configuração, telemetria CSV, perfil, propostas e relatórios em `resultados/sessao-NNN`. O JSON de cada tentativa guarda semente, tempo, resultado e perfil usado. A pista pode ser reaberta. As etapas da sessão são conduzidas em memória enquanto a janela fica aberta.

### Histórico de iterações

Depois de **Gerar mapa**, clique em **Otimizar automaticamente** para delegar as iterações ao simulador. Ele usa a validação de treino e a proposta já preparadas, quando disponíveis. Se precisar de uma referência, faz a primeira validação. Em seguida, diagnostica, aplica uma proposta e testa o perfil. O botão fica disponível também após várias iterações manuais.

Cada passada automática usa o mesmo passo de simulação e a mesma confirmação pareada do treino manual. A pista mostra a trajetória ao término de cada passada. O ciclo avança assim que o cálculo e a confirmação terminam. A barra de status informa a iteração e quantas propostas foram testadas e aceitas desde o último clique em **Otimizar automaticamente**. O histórico completo e as telemetrias ficam disponíveis na sessão. O ZIP identifica essas validações com `validacao_automatica`.

**Pausar otimização** conclui a validação em andamento, inclusive a confirmação, e conserva a referência. Para retomar, clique em **Otimizar automaticamente** novamente. A busca encerra quando o diagnóstico esgota as propostas ou volta a sugerir um perfil já rejeitado contra a mesma referência. A tomada oficial continua disponível pelo botão **Tomar tempo**.

A faixa abaixo dos botões e a barra lateral da pista mostram a iteração atual, a melhor referência do mapa e o resultado da última comparação. Em janelas baixas, use a rolagem da barra lateral para chegar aos indicadores sem mover a pista. Um delta negativo em segundos indica uma volta mais rápida. Enquanto as voltas estiverem incompletas, um delta positivo em metros indica que o robô avançou mais. O botão **Ver histórico** abre todas as tentativas da sessão.

O **delta ao vivo**, abaixo do círculo de tração, compara o tempo da passagem atual com o da referência na mesma posição ao longo da pista. O número aparece em segundos, com três casas decimais. Valores positivos em vermelho indicam atraso e negativos em verde indicam vantagem. O treino compara com a melhor referência do mapa. A tomada Pro compara com a melhor volta oficial válida da mesma tomada, entre os portões. Se a referência de treino terminar antes da passagem atual, o indicador informa que ela foi ultrapassada.

A aba **Histórico** acompanha cada validação, se ela já foi diagnosticada e a avaliação das propostas aplicadas. A primeira validação do mapa vira referência. **Aplicar proposta** cria uma iteração pendente e a próxima validação testa esse perfil com a mesma semente de ruído. O histórico mostra a referência usada, o tempo, a diferença entre voltas completas, a distância medida pelos encoders e a porcentagem de amostras sem linha. A lista permanece disponível até fechar a janela, inclusive se você trocar de pista.

Enquanto o robô estiver recuperando a linha, o algoritmo prioriza completar a volta com o corpo sobre a faixa. Entre duas voltas incompletas, o primeiro critério é a distância percorrida. A primeira volta de treino competitiva vira a referência para a busca de tempo.

Para comparar duas passagens, a parcial de uma curva começa na entrada e termina até 50 cm depois da saída, limitada pela entrada da curva seguinte. A velocidade de saída é medida ao final da própria curva. As retas também ganham parciais próprias, que podem se sobrepor à janela de saída. Um delta local negativo indica ganho nesse intervalo. Uma janela atravessada apenas parcialmente fica sem tempo comparável.

Em cada volta competitiva, a busca seleciona uma região e um parâmetro. Com margem de tração e velocidade nominal atingida, pode elevar o alvo em até 7%. O diagnóstico pode propor antecipação quando o giro atrasa na entrada ou quando o QTR permanece deslocado no sentido da curva, com margem lateral e sentido compatível no mapa. Inversões repetidas do erro do QTR em reta podem motivar um ajuste em Kd. Uma tentativa rejeitada de ajuste isolado orienta um passo intermediário na mesma região.

Ao selecionar uma curva em **Diagnóstico → Curvas e saídas**, compare o pico de tração nos 100 ms anteriores à perda do QTR com a demanda lateral e o tempo de frenagem na recuperação. Assim é possível verificar se a demanda já era alta antes da perda da linha. O P95 da tração combinada inclui a passagem inteira pela região, inclusive as frenagens. O CSV por curva registra essas medidas junto da hipótese e do ajuste proposto.

Uma proposta que supera a referência competitiva na triagem passa pela confirmação automática. Referência e candidata usam as sementes 7, 37 e 83. O simulador reaproveita a triagem e as medições anteriores da mesma referência. Para promover a proposta, exige três voltas competitivas da candidata e mediana negativa dos deltas entre pares com duas voltas competitivas. O **Histórico** e o ZIP mostram o resultado da confirmação. Em caso de rejeição, o perfil anterior volta a ser usado. A tomada Pro mantém a classificação pelo menor tempo válido entre os portões.

Clique em **Exportar sessão** para salvar um ZIP compartilhável da pista atual. Ao trocar de pista ou iniciar um desenho, o próximo teste começa uma nova sessão de exportação. Os arquivos das pistas anteriores ficam em `resultados` e suas iterações continuam visíveis no histórico da janela. O `resumo-da-sessao.json` liga cada iteração da pista atual à referência, à telemetria, ao diagnóstico e à proposta aplicada. Um delta de tempo positivo indica piora em relação à referência.

## Ferramentas offboard

Também podem processar CSVs de fora da interface, respeitando as mesmas colunas e unidades:

```powershell
py -3.13 offboard.py --robot robo.json mapa 01-mapeamento.csv mapa.json
py -3.13 offboard.py --robot robo.json diagnostico 02-validacao.csv mapa.json proposta.json relatorio.csv
py -3.13 offboard.py --robot robo.json diagnostico 03-validacao.csv perfil-testado.json proposta.json relatorio.csv --pista pista.json --referencia 02-validacao.csv
```

Com `--pista`, o diagnóstico também gera `relatorio-curvas.csv` e usa a busca por região. `--referencia` aceita uma passada do mesmo mapa para calcular os deltas locais. Passe `--incompleta` para uma tentativa abortada. O peso do giroscópio no mapeamento varia de 0 a 1 e o padrão é 0,75. Use a configuração de robô que gerou a telemetria.

- `t_s` é o tempo desde a largada em segundos. `enc_left` e `enc_right` são contagens acumuladas desde zero. `gyro_z_rad_s` mede a rotação no plano. `acc_x_m_s2`, `acc_y_m_s2` e `acc_z_m_s2` são acelerações nos eixos longitudinal, lateral e vertical.
- Validação e tomada de tempo incluem `ir_0` a `ir_7`, com intensidade normalizada de linha, `line_error_m` (vazio na perda de linha), os indicadores 0/1 `line_edge`, `line_lost`, `body_on_line` e `motor_sat`, e `target_m_s`. `line_lost` é a perda detectada pelo QTR. `body_on_line` indica se a caixa do robô ainda toca a faixa simulada.
- `profile_target_m_s` é a velocidade nominal do setor, `planned_m_s` é o teto calculado com antecipação da frenagem e `target_m_s` é o alvo aplicado ao controlador após a rampa e as regras de recuperação ou parada. O painel lateral mostra os três valores durante a validação. Aumentar o RPM eleva o teto do motor, mas a aderência, a distância para frear e a rampa de aceleração ainda podem limitar a velocidade.
- `encoder_speed_m_s` é a velocidade filtrada a partir dos encoders. `steering_kp` é o ganho proporcional aplicado naquela amostra.
- `lateral_grip_pct` registra a demanda de aderência em porcentagem. `lateral_grip_limited` vale 1 nos instantes em que o giro ficou limitado.
- `traction_usage_pct` registra a demanda combinada do círculo. `traction_long_m_s2` e `lateral_demand_m_s2` são as componentes com sinal em m/s².
- `lateral_limit_m_s2` registra o limite lateral efetivo. `suction_force_n` é a força aplicada no teste e vale zero com o sistema desligado.
- `track_progress_m` registra a posição longitudinal simulada ao final de cada amostra. O diagnóstico usa essa medida para comparar passagens no mesmo ponto. Em CSVs antigos estima a posição pelos encoders e registra o método em `alinhamento`. O controlador localiza os setores pela odometria. `yaw_feedforward_rad_s` registra o comando de giro antecipado aplicado.

## Hipóteses do modelo

A configuração usa rodas de 32 mm, motores N20 de até 1500 rpm e uma placa QTR de aproximadamente 65 mm com oito sensores. Tratando os 1500 rpm como rotação máxima no eixo da roda, o teto linear teórico é de 2,51 m/s por roda. Com 5 mm de margem estimada em cada ponta da placa, o simulador usa 55 mm entre o primeiro e o último sensor, cerca de 7,9 mm entre vizinhos. Esses valores podem ser ajustados em `simulator.py`. Um `robo.json` de sessão antiga com `wheel_limit` numérico preserva o limite de velocidade daquela sessão nas ferramentas offboard.

A planta do robô indica 180 mm de largura externa, 60 mm de profundidade do chassi e 80 mm de extensão frontal até a linha dos sensores. O modelo estima o eixo a 15 mm da traseira do chassi (`axle_from_rear`), o que coloca o QTR a 125 mm do eixo. A checagem Pro inclui as rodas e a placa dianteira no comprimento total e na verificação de parada.

A bitola estimada é de 164 mm entre os centros das rodas motrizes, considerando pneus de 16 mm de largura e a cota externa de 180 mm. Os encoders de 360 contagens por volta da roda são uma hipótese de bancada. Os motores têm saturação e atraso de primeira ordem. Um limite lateral aproxima a perda de aderência. A IMU é um modelo planar com viés no giroscópio e ruído. O acelerômetro fica gravado para diagnóstico e fora da integração do mapa. A leitura de refletância aproxima a intensidade medida em cada posição do array.

A proposta de ajuste é heurística. Ela reduz a velocidade nos trechos com perda, borda ou limite mecânico, altera os ganhos conforme o sintoma e tenta aumentar a velocidade nos trechos estáveis. O modelo serve para comparar estratégias e conferir o circuito de dados. A capacidade real dos N20, a resposta do GY-BM008X, a aderência e os ganhos dependem de calibração na bancada física. O mapa mostra a deriva de odometria, inclusive a distância entre o início e o fim estimados.
