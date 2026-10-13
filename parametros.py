import numpy as np

# ==================================================================================
# PARÂMETROS ELETROMECÂNICOS ATUALIZADOS (NÃO MUDAR - IDENTIFICADOS EM BANCADA/PLACA)
# ==================================================================================
CONSTANTE_VELOCIDADE_GERADOR_V_RAD_S = 2.675717 
OFFSET_FEM = 9.5104 
TORQUE_MAXIMO_MOTOR_NM = 79.63 
COEFICIENTE_ATRITO_GERADOR = 0.001885
TSR_IDEAL = 7.5
TAXA_VARIACAO_RPM_MAX = 100

# ==================================================================================

# chutes
RPM = [0, 500, 550, 600, 700, 900, 1000]
GANHOS = [1/25, 1/25, 1/18, 1/15, 1/1.5, 1/1, 1/0.5]
TABELA_GANHO_ARMADURA = dict(zip(RPM, GANHOS))
TAU_ARMADURA = 0.03
CAPACITANCIA_BARRAMENTO_FARADAY = 0.007
INERCIA_GERADOR_KG_M2 = 2 * COEFICIENTE_ATRITO_GERADOR
# curva otima ajustada pelo main_gemeo_pid
pontos_v_inversor_hibrido = np.array([69.8629985210623, 93.44977714451827, 117.03655576797424, 140.6233343914302, 164.2101130148862, 187.79689163834215, 211.3836702617981, 234.9704488852541, 258.55722750871007, 282.14400613216606, 305.730784755622, 329.31756337907797])
pontos_i_inversor_hibrido = np.array([3.536691111355813, 3.542729586728779, 3.7740709022210797, 4.230715057832715, 4.912662053563685, 5.819911889413989, 6.952464565383626, 8.3103200814726, 9.893478437680908, 11.701939634008552, 13.735703670455527, 15.994770547021838])

pontos_r_inversor_hibrido = pontos_v_inversor_hibrido/pontos_i_inversor_hibrido
# =============================================================================

# apenas para isolar o comportamento do motor/gerador de bancada,
COEFICIENTE_ATRITO_TURBINA = 0.0
INERCIA_TURBINA_KG_M2 = 0.0

DENSIDADE_AR_KG_M3 = 1.2754
RAIO_TURBINA_METROS = 2.5 # escolha
RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS = 4 # escolha
VELOCIDADE_MAXIMA_PAS_GRAUS_S = 5.0
TENSAO_MAXIMA_INVERSOR_V = pontos_v_inversor_hibrido[-1] # tabela inversor hibrido
TENSAO_MINIMA_INVERSOR_V = pontos_v_inversor_hibrido[0]  # tabela inversor hibrido
TEMPO_ESPERA_LIGAR = 5.0  # Tempo necessário (em segundos) com tensão alta para ligar
POTENCIA_MECANICA_MAXIMA_W = 3000 # escolha
CORRENTE_ARAMDURA_MAXIMA_A = 15

PASSO_SIMULACAO_SEGUNDOS = 0.004 
PASSO_HARDWARE_SEGUNDOS = 0.01 # deve ser maior que a latencia da comunicacao
TEMPO_TOTAL_EMULACAO_SEGUNDOS = 100 # escolha

TAXA_ATUALIZACAO_GRAFICOS_SEGUNDOS = 1.0 #escolha

CONSTANTE_TEMPO_FILTRO_SENSOR_TORQUE = 0.1
CONSTANTE_TEMPO_FILTRO_U_INVERSOR = 0.4
CONSTANTE_TEMPO_FILTRO_VELOCIDADE_ALVO = 0.5
CONSTANTE_TEMPO_FILTRO_FORCAELETROMOTRIZ = 0.0

#PERFIL_VENTO = 'CONSTANTE'
#PERFIL_VENTO = 'ESCADA' 
#PERFIL_VENTO = 'NATURAL'
PERFIL_VENTO = 'RAMPA'
from numpy import load
VETOR_VENTO_M_S = load('vetor_vento.npy')
vento_constante = 5
