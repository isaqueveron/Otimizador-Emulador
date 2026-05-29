# 🌬️ Simulador de Otimização MPPT para Turbina Eólica

Este projeto implementa um ambiente de simulação para avaliação e otimização de estratégias de **Maximum Power Point Tracking (MPPT)** aplicadas a uma turbina eólica virtual.

O sistema compara uma configuração de MPPT **original de fábrica** com uma curva **otimizada dinamicamente** gerada por simulação utilizando controle PID.

---

# 🚀 Visão Geral

O simulador modela o comportamento eletromecânico de uma turbina eólica e executa um processo em **três fases principais** para determinar a relação ideal entre tensão e corrente (`V x I`) visando a máxima extração de potência.

## 🎯 Objetivos

* Otimizar parâmetros MPPT usando controle PID
* Aprender automaticamente a curva ideal de carga da turbina
* Comparar desempenho entre configuração original e otimizada
* Avaliar eficiência aerodinâmica (`Cp`)
* Gerar gráficos comparativos de potência, RPM e rastreamento de desempenho

---

# 🛠️ Pré-requisitos

O projeto requer Python 3 e as seguintes bibliotecas:

* `numpy`
* `matplotlib`

A biblioteca `dataclasses` já está incluída nas versões modernas do Python.

## 📦 Instalação

```bash
pip install numpy matplotlib
```

---

# 📁 Estrutura do Projeto

Certifique-se de que os seguintes arquivos estejam no mesmo diretório:

```text
.
├── main.py
├── modelo_aerogerador.py
├── pid_module.py
└── README.md
```

## 📌 Descrição dos Módulos

### `modelo_aerogerador.py`

Contém a implementação da classe:

```python
TurbinaVirtual
```

Responsável pela modelagem física e eletromecânica da turbina.

### `pid_module.py`

Contém a implementação da classe:

```python
PIDController
```

Responsável pelo controle dinâmico de carga e rastreamento do ponto ótimo de operação.

---

# 🏗️ Arquitetura da Simulação

A execução é dividida em três fases:

---

## 🔹 Fase 1 — Otimização MPPT

Um controlador PID regula dinamicamente a carga elétrica da turbina para manter a:

* TSR ideal (*Tip Speed Ratio*)
* máxima eficiência aerodinâmica (`Cp`)

Durante essa etapa:

* os pontos ótimos de operação são registrados;
* uma nova curva `U x I` é derivada;
* é realizada regressão polinomial para aproximar a curva ótima.

---

## 🔹 Fase 2 — Linha de Base (Original)

A simulação é executada utilizando:

* parâmetros MPPT originais;
* curva de hardware padrão (*hardcoded*).

Essa etapa serve como referência de desempenho.

---

## 🔹 Fase 3 — Configuração Otimizada

A nova curva obtida na Fase 1 substitui os parâmetros originais.

O sistema então calcula:

* potência gerada;
* energia acumulada;
* eficiência comparativa;
* ganho percentual de desempenho.

---

# ▶️ Como Executar

Execute o script principal:

```bash
python main.py
```

---

# ⚙️ Parâmetros da Simulação

Durante a execução, o terminal solicitará:

| Parâmetro                  | Descrição                                       |
| -------------------------- | ----------------------------------------------- |
| **Raio da Turbina**        | Raio das pás em metros                          |
| **Relação de Transmissão** | Relação da caixa multiplicadora                 |
| **Tempo de Emulação**      | Duração total da simulação                      |
| **Perfil de Vento**        | Tipo de vento: `RAMPA`, `ESCADA` ou `CONSTANTE` |

---

# 📊 Resultados Gerados

Após a simulação, o sistema fornece:

## 📋 Logs no Console

* Energia total produzida (`Wh`)
* Comparação entre:

  * configuração original
  * configuração otimizada

---

## 📈 Gráficos Matplotlib

### 🔸 Curvas MPPT

Comparação entre:

* curva original
* curva otimizada (`V x I`)

---

### 🔸 Rastreamento Cinético

Comparação entre:

* RPM real
* RPM alvo ideal

---

### 🔸 Potência de Saída

Comparação direta da potência gerada ao longo do tempo.

---

### 🔸 Coeficiente de Desempenho (`Cp`)

Avaliação da eficiência aerodinâmica da turbina em relação ao:

```text
Limite de Betz ≈ 0.593
```

---

# 🧠 Conceitos Utilizados

* MPPT (*Maximum Power Point Tracking*)
* Controle PID
* TSR (*Tip Speed Ratio*)
* Regressão Polinomial
* Modelagem Eletromecânica
* Eficiência Aerodinâmica
* Simulação Dinâmica

---

# 💡 Solução de Problemas

## ❌ Erros na Geração da Curva

Caso a Fase 1 falhe ao gerar a curva otimizada:

* o sistema retorna automaticamente para:

  * `V_ORIGINAL`
  * `R_ORIGINAL`

Verifique os parâmetros:

```python
threshold_I
threshold_V
```

na função:

```python
extrair_curva_otima_com_filtro()
```

Os valores devem ser compatíveis com a escala elétrica do hardware modelado.

---

## ❌ ModuleNotFoundError

Certifique-se de que os arquivos:

* `modelo_aerogerador.py`
* `pid_module.py`

estejam:

* no mesmo diretório do `main.py`; ou
* configurados no `PYTHONPATH`.

---

# 🔬 Aplicações

Este projeto pode ser utilizado para:

* pesquisa acadêmica;
* análise de eficiência energética;
* desenvolvimento de algoritmos MPPT;
* testes de controle PID;
* estudo de pequenas turbinas eólicas.

---

# 📜 Licença

Este projeto é destinado a fins educacionais e de pesquisa.
