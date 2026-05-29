# Simulador de Otimização MPPT para Turbina Eólica

Este projeto é um ambiente de simulação desenvolvido para avaliar e 
otimizar estratégias de Maximum Power Point Tracking (MPPT) para uma 
turbina eólica virtual. Ele compara uma configuração de MPPT "padrão de 
fábrica" contra uma curva otimizada dinamicamente gerada via simulação 
de controle PID.

## 🚀 Visão Geral

O sistema modela o comportamento eletromecânico de uma turbina eólica e 
executa um processo de simulação em três fases para determinar a relação 
ideal entre a tensão e a corrente da turbina para extração máxima de energia.

### Objetivos Principais

* Otimização de Parâmetros: Uso de um controlador PID para "aprender" a 
  curva de carga ideal (V vs. I) para o hardware específico da turbina.
* Benchmarking de Desempenho: Comparar a eficiência (Cp) e a saída de 
  energia total (Wh) do hardware em relação aos parâmetros originais.
* Análise Visual: Geração de gráficos comparativos para visualizar 
  potência, RPM e rastreamento do coeficiente de desempenho (Cp).

## 🛠️ Pré-requisitos

Para executar esta simulação, certifique-se de que seu ambiente Python 
esteja configurado com as seguintes dependências:

* numpy
* matplotlib
* dataclasses (Biblioteca padrão)

Você também precisará dos dois módulos personalizados referenciados no 
script no mesmo diretório:
* modelo_aerogerador.py (Contém a classe TurbinaVirtual)
* pid_module.py (Contém a classe PIDController)

### Instalação

```bash
pip install numpy matplotlib
🏗️ Arquitetura do Projeto
A simulação segue uma lógica de execução estruturada em 3 fases:

Fase 1 (Otimização): Um PIDController regula ativamente a carga para
manter a Razão de Velocidade da Ponta (TSR) ideal. O sistema registra
esses estados "ideais" e deriva uma nova curva U x I usando regressão
polinomial.

Fase 2 (Linha de Base): A simulação é executada usando os parâmetros
de hardware originais (hardcoded) para estabelecer uma base de
desempenho.

Fase 3 (Otimizada): A simulação é executada novamente, substituindo
os parâmetros originais pela curva derivada na Fase 1 para calcular
os ganhos potenciais de energia.

🏃‍♂️ Como Executar
Certifique-se de que todos os arquivos do projeto estejam no mesmo
diretório.

Execute o script principal:

Bash
python main.py
Siga as instruções interativas no terminal:

Raio da Turbina: Especifique o raio da turbina em metros.

Relação de Transmissão: Defina a relação da caixa de engrenagens.

Tempo de Emulação: Duração total da simulação em segundos.

Perfil de Vento: Escolha entre RAMPA, ESCADA ou CONSTANTE.

📊 Resultados
Após a conclusão, o script fornece:

Logs no Console: Energia total produzida (em Wh) tanto para a
configuração original quanto para a otimizada.

Gráficos Matplotlib:

Curvas MPPT: Comparação U x I.

Rastreamento Cinético: Comparação de RPM contra o alvo ideal.

Potência de Saída: Comparação direta de geração de energia.

Coeficiente de Desempenho (Cp): Análise de eficiência em comparação
ao limite teórico de Betz (Cp, max ≈ 0.593).

💡 Solução de Problemas
Erros de Fitting: Se a Fase 1 falhar ao gerar uma curva, o script
retorna para V_ORIGINAL e R_ORIGINAL. Certifique-se de que os valores
de threshold_I e threshold_V na função extrair_curva_otima_com_filtro
sejam apropriados para a escala do seu hardware.

Dependências: Se receber um erro ModuleNotFoundError, verifique se
modelo_aerogerador.py e pid_module.py estão localizados no seu
PYTHONPATH ou no mesmo diretório do script principal.

Este projeto serve como uma ferramenta de pesquisa para avaliar a
eficiência de pequenas turbinas eólicas e o ajuste de controladores MPPT.
