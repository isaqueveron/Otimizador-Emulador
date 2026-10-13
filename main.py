import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass

# Para evitar erros de importação se rodado diretamente, mantive suas chamadas originais:
from modelo_aerogerador import TurbinaVirtual
from pid_module import PIDController

# ==================================================================================
# PARÂMETROS FIXOS (Eletromecânicos da Bancada/Hardware)
# ==================================================================================
CONST_VEL = 2.675717
OFFSET_FEM = 9.5104 
TORQUE_MAX = 79.63 
COEF_ATRITO_GERADOR = 0.001885
TSR_IDEAL = 7.5
TAXA_RPM_MAX = 100
TAU_ARMADURA = 0.03
CAPACITANCIA_BARRAMENTO = 0.007
INERCIA_GERADOR = 2 * COEF_ATRITO_GERADOR
COEF_ATRITO_TURBINA = 0.0
INERCIA_TURBINA = 0.0
DENSIDADE_AR = 1.2754
VEL_MAX_PITCH = 5.0
TEMPO_ESPERA_LIGAR = 5.0
PASSO_SIMULACAO = 0.004 

TABELA_GANHO_ARMADURA = {0: 1/25, 500: 1/25, 550: 1/18, 600: 1/15, 700: 1/1.5, 900: 1/1, 1000: 1/0.5}

V_ORIGINAL = np.array([90, 110, 130, 150, 170, 190, 210, 230, 250, 270, 290, 310])
I_ORIGINAL = np.array([0.001, 1.5, 3.0, 4.5, 6.0, 7.5, 9.0, 10.5, 12.0, 13.5, 15.0, 16.5]) / 2
R_ORIGINAL = V_ORIGINAL / I_ORIGINAL

@dataclass
class ConfigEnsaio:
    raio: float
    relacao: float
    tempo: float
    perfil: str
    v_constante: float

class RegistroDeSimulacao:
    def __init__(self, duracao_s, passo_simulacao):
        self.n_sim = int(duracao_s / passo_simulacao) + 1
        self.idx_sim = 0
        self.tempo = np.zeros(self.n_sim)
        self.vento = np.zeros(self.n_sim)
        self.rpm = np.zeros(self.n_sim)
        self.rpm_ideal = np.zeros(self.n_sim)
        self.tensao = np.zeros(self.n_sim)
        self.corrente = np.zeros(self.n_sim)
        self.pot_entregue = np.zeros(self.n_sim)
        self.cp = np.zeros(self.n_sim)
        self.tensoes_barramento_volts = np.zeros(self.n_sim)

def extrair_curva_otima_grau_n(registros, grau_n=10):
    s = registros.idx_sim
    V = registros.tensoes_barramento_volts[:s]
    I = registros.corrente[:s]

    novo_threshold_I = 0.001  
    novo_threshold_V = 30.0 
    
    limite_superior_V = 350.0
    limite_superior_I = 16.5

    mask = (V > novo_threshold_V) & (I > novo_threshold_I)
    V_fit = V[mask]
    I_fit = I[mask]

    if len(V_fit) < (grau_n + 1):
        print(f"Aviso: Pontos insuficientes após o filtro ({len(V_fit)}) para o grau {grau_n}.")
        return None, None, None

    try:
        coeffs = np.polyfit(V_fit, I_fit, grau_n)
        p = np.poly1d(coeffs)
        v_max_ajuste = min(V_fit.max(), limite_superior_V)
        v_opt = np.linspace(V_fit.min(), v_max_ajuste, 12)
        i_opt = p(v_opt)
        i_opt = np.clip(i_opt, 0.0, limite_superior_I)
        
        with np.errstate(divide='ignore', invalid='ignore'):
            r_opt = v_opt / i_opt
        
        r_opt[np.isinf(r_opt)] = 1e50
        r_opt[np.isnan(r_opt)] = 1e50

        return v_opt, i_opt, r_opt

    except Exception as e:
        print(f"Erro no fitting livre de grau {grau_n}: {e}")
        return None, None, None
    

def obter_vento(tempo, perfil, v_constante, tempo_total, v_pico=5.0):
    if perfil == 'CONSTANTE': 
        return v_constante
        
    if perfil == 'RAMPA':
        meio = tempo_total / 2.0
        if 0.0 <= tempo < meio:
            return (v_pico / meio) * tempo
        elif meio <= tempo <= tempo_total:
            return v_pico - (v_pico / (tempo_total - meio)) * (tempo - meio)
        return 0.0
        
    if perfil == 'ESCADA':
        n_degraus = 5
        passo = tempo_total / n_degraus
        degrau = int(tempo / passo)
        velocidades = [2.0, 4.0, 6.0, 8.0, v_pico]
        
        if 0 <= degrau < n_degraus:
            return velocidades[degrau]
        return 0.0
        
    return 0.0

def executar_fase_pid(config: ConfigEnsaio):
    print("\n[FASE 1/3] Rodando Controle PI para encontrar curva ótima...")
    turbina = TurbinaVirtual(
        constante_velocidade=CONST_VEL, offset_fem=OFFSET_FEM, tabela_ganho_armadura=TABELA_GANHO_ARMADURA, 
        tau_armadura=TAU_ARMADURA, capacitancia_barramento=CAPACITANCIA_BARRAMENTO,
        raio_turbina=config.raio, inercia_turbina=INERCIA_TURBINA, inercia_gerador=INERCIA_GERADOR, 
        coeficiente_atrito_turbina=COEF_ATRITO_TURBINA, coeficiente_atrito_gerador=COEF_ATRITO_GERADOR,
        taxa_variacao_rpm_maxima=TAXA_RPM_MAX, velocidade_angular_maxima_pitch=VEL_MAX_PITCH, 
        torque_maximo_motor=TORQUE_MAX, tsr_ideal=TSR_IDEAL, relacao_transmissao_caixa_engrenagens=config.relacao,
        modelo_coeficiente_potencia='classico', densidade_ar_kg_m3=DENSIDADE_AR, sinal_controle_mppt='corrente de carga'
    )

    pid = PIDController(kp=2.0, ki=1.0, kd=0.0, out_min=0, out_max=16.5)
    reg = RegistroDeSimulacao(config.tempo, PASSO_SIMULACAO)
    t = 0.0
    corrente_filtro = 0.0

    while t < config.tempo:
        v_vento = obter_vento(t, config.perfil, config.v_constante, config.tempo)
        omega_alvo = (v_vento * TSR_IDEAL / config.raio)
        
        u = pid.compute(turbina.get_velocidade_angular_turbina_rad_s(), omega_alvo, PASSO_SIMULACAO, True)
        corrente_filtro = (1 - 0.1) * corrente_filtro + 0.1 * u
        turbina.set_flag_vazio(u == 0.0)
        
        turbina.executar_passo_simulacao(v_vento, 0, PASSO_SIMULACAO, corrente_carga=corrente_filtro)
        
        if reg.idx_sim < reg.n_sim:
            reg.tempo[reg.idx_sim] = t
            reg.tensao[reg.idx_sim] = turbina.get_tensao_capacitor_volts()
            reg.tensoes_barramento_volts[reg.idx_sim] = turbina.get_tensao_capacitor_volts()
            reg.pot_entregue[reg.idx_sim] = turbina.get_potencia_eletrica_entregue_w()
            reg.corrente[reg.idx_sim] = corrente_filtro
            reg.idx_sim += 1
        t += PASSO_SIMULACAO

    energia_wh = (np.sum(reg.pot_entregue[:reg.idx_sim]) * PASSO_SIMULACAO) / 3600.0
    print(f">> Fase PI Concluída. Energia Total: {energia_wh:.3f} Wh")

    v_opt, i_opt, r_opt = extrair_curva_otima_grau_n(reg)

    if v_opt is None:
        print("Erro: Falha na extração. Usando valores padrão.")
        return V_ORIGINAL, I_ORIGINAL, R_ORIGINAL, reg
    
    return v_opt, i_opt, r_opt, reg

def executar_malha_aberta(config: ConfigEnsaio, v_tabela, r_tabela, nome_fase):
    print(f"\n[{nome_fase}] Rodando Malha Aberta...")
    turbina = TurbinaVirtual(
        constante_velocidade=CONST_VEL, offset_fem=OFFSET_FEM, tabela_ganho_armadura=TABELA_GANHO_ARMADURA, 
        tau_armadura=TAU_ARMADURA, capacitancia_barramento=CAPACITANCIA_BARRAMENTO,
        raio_turbina=config.raio, inercia_turbina=INERCIA_TURBINA, inercia_gerador=INERCIA_GERADOR, 
        coeficiente_atrito_turbina=COEF_ATRITO_TURBINA, coeficiente_atrito_gerador=COEF_ATRITO_GERADOR,
        taxa_variacao_rpm_maxima=TAXA_RPM_MAX, velocidade_angular_maxima_pitch=VEL_MAX_PITCH, 
        torque_maximo_motor=TORQUE_MAX, tsr_ideal=TSR_IDEAL, relacao_transmissao_caixa_engrenagens=config.relacao,
        modelo_coeficiente_potencia='classico', densidade_ar_kg_m3=DENSIDADE_AR, sinal_controle_mppt='resistencia de carga'
    )

    reg = RegistroDeSimulacao(config.tempo, PASSO_SIMULACAO)
    t = 0.0
    tempo_acima_min = 0.0
    conectado = False
    v_min = v_tabela[0]

    while t < config.tempo:
        v_vento = obter_vento(t, config.perfil, config.v_constante, config.tempo)
        fem = turbina.get_forca_eletromotriz()
        
        if fem >= v_min:
            tempo_acima_min += PASSO_SIMULACAO
            if tempo_acima_min >= TEMPO_ESPERA_LIGAR: conectado = True
        else:
            tempo_acima_min = 0.0
            conectado = False

        r_carga = np.interp(turbina.get_tensao_capacitor_volts(), v_tabela, r_tabela, left=1e50) if conectado else 1e50
        turbina.set_flag_vazio(not conectado)
        
        turbina.executar_passo_simulacao(v_vento, 0, PASSO_SIMULACAO, r_carga) 
        
        if reg.idx_sim < reg.n_sim:
            reg.tempo[reg.idx_sim] = t
            reg.vento[reg.idx_sim] = v_vento
            reg.rpm[reg.idx_sim] = turbina.get_velocidade_angular_gerador_rad_s() * 60 / (2*np.pi)
            reg.rpm_ideal[reg.idx_sim] = (v_vento * TSR_IDEAL / config.raio) * config.relacao * 60 / (2*np.pi)
            reg.tensao[reg.idx_sim] = turbina.get_tensao_capacitor_volts()
            reg.tensoes_barramento_volts[reg.idx_sim] = turbina.get_tensao_capacitor_volts()
            reg.corrente[reg.idx_sim] = turbina.get_corrente_armadura_amperes()
            reg.pot_entregue[reg.idx_sim] = turbina.get_potencia_eletrica_entregue_w()
            
            p_disp = 0.5 * DENSIDADE_AR * (np.pi * config.raio**2) * (v_vento**3) if v_vento > 0 else 0.01
            reg.cp[reg.idx_sim] = turbina.get_potencia_absorvida_vento_w() / p_disp
            reg.idx_sim += 1

        t += PASSO_SIMULACAO

    energia_wh = (np.sum(reg.pot_entregue[:reg.idx_sim]) * PASSO_SIMULACAO) / 3600.0
    print(f">> Concluído. Energia Total: {energia_wh:.3f} Wh")
    return reg, energia_wh

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

# =============================================================================
# ESTILO GLOBAL (mesmo do gerar_plots.py do artigo)
# =============================================================================
def configurar_estilo():
    rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
        "mathtext.fontset": "cm",
        "font.size": 10,
        "axes.titlesize": 10,
        "axes.labelsize": 9.5,
        "legend.fontsize": 8,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "lines.linewidth": 1.2,
        "axes.grid": True,
        "grid.linewidth": 0.5,
        "grid.alpha": 0.5,
        "axes.linewidth": 0.8,
        "legend.frameon": True,
        "legend.framealpha": 0.9,
        "legend.edgecolor": "0.8",
        "legend.borderpad": 0.4,
        "legend.handlelength": 1.6,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.03,
    })

# Paleta (a mesma do artigo)
COR_ORIGINAL   = "#1F5FBF"   # azul: tabela de fábrica (mesmo tom de "Referência")
COR_OTIMIZADA  = "#2E7D32"   # verde: tabela otimizada (mesmo tom do Gêmeo em potência)
COR_VARREDURA  = "0.65"      # cinza: dados brutos da varredura PI
COR_SENSOR     = "black"


def _fig_width(largura_relativa=1.0):
    largura_textwidth_cm = 16.5
    return (largura_textwidth_cm * largura_relativa) / 2.54


def _fmt(x, casas=2):
    """Formata com vírgula decimal (padrão do texto do artigo)."""
    return f"{x:.{casas}f}".replace(".", ",")


def plotar_otimizador(reg_orig, reg_opt, reg_pid, en_orig, en_opt,
                      v_opt, i_opt, v_original, i_original,
                      nome_arquivo="otimizador_energia_iv.pdf"):
    """
    Figura de 2 painéis no estilo do artigo:
      (esq.) curva V x I: varredura PI, tabela original e tabela otimizada (12 pontos)
      (dir.) potência elétrica útil no tempo, com a energia acumulada [Wh] na legenda
    """
    configurar_estilo()
    s = reg_orig.idx_sim
    t = reg_orig.tempo[:s]
    s_pid = reg_pid.idx_sim
    ganho = 100.0 * (en_opt / en_orig - 1.0)

    fig, (ax_iv, ax_pot) = plt.subplots(
        1, 2, figsize=(_fig_width(1.0), 2.8),
        gridspec_kw={"width_ratios": [1, 1.15]}, constrained_layout=True
    )

    # --- Painel 1: Corrente x Tensão ---------------------------------------
    passo = 5  # subamostra a varredura para manter o PDF leve
    ax_iv.scatter(reg_pid.tensoes_barramento_volts[:s_pid:passo], reg_pid.corrente[:s_pid:passo],
                  s=1.5, color=COR_VARREDURA, alpha=0.5, rasterized=True, label="Varredura PI")
    ax_iv.plot(v_original, i_original, color=COR_ORIGINAL, marker="o", ms=3.2, lw=1.2, label="Original")
    ax_iv.plot(v_opt, i_opt, color=COR_OTIMIZADA, marker="s", ms=3.2, lw=1.2, label="Otimizada")
    ax_iv.set_xlabel("Tensão [V]")
    ax_iv.set_ylabel("Corrente [A]")
    ax_iv.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=3)

    # --- Painel 2: Potência útil e energia ---------------------------------
    ax_pot.plot(t, reg_orig.pot_entregue[:s], color=COR_ORIGINAL, lw=1.1,
                label=f"Original ({_fmt(en_orig)} Wh)")
    ax_pot.plot(t, reg_opt.pot_entregue[:s], color=COR_OTIMIZADA, lw=1.1,
                label=f"Otimizada ({_fmt(en_opt)} Wh)")
    ax_pot.fill_between(t, reg_orig.pot_entregue[:s], reg_opt.pot_entregue[:s],
                        color=COR_OTIMIZADA, alpha=0.15, lw=0)
    ax_pot.set_xlabel("Tempo [s]")
    ax_pot.set_ylabel("Potência útil [W]")
    ax_pot.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2)

    fig.align_ylabels()
    fig.savefig(nome_arquivo, format="pdf")
    plt.close(fig)
    print(f"Figura salva em: {nome_arquivo}  (ganho de energia: {_fmt(ganho)}%)")

if __name__ == "__main__":
    print("======================================================")
    print("   AVALIADOR DE MAXIMA TRANSFERENCIA DE ENERGIA")
    print("======================================================")
    
    try:
        r_input = float(input("1. Raio da Turbina [m] (Ex: 3.0): ") or 3.0)
        rel_input = float(input("2. Relação de Transmissão (Ex: 6.0): ") or 6.0)
        t_input = float(input("4. Tempo de Emulação [s] (Ex: 250): ") or 250.0)
        
        print("\n5. Perfil de Vento:")
        print("   [1] RAMPA")
        print("   [2] ESCADA")
        print("   [3] CONSTANTE")
        p_escolha = input("   Opção (Padrão: RAMPA): ")
        
        mapa_perfil = {'2': 'ESCADA', '3': 'CONSTANTE'}
        perfil = mapa_perfil.get(p_escolha, 'RAMPA')
        
        v_const = 5.0
        if perfil == 'CONSTANTE':
            v_const = float(input("   Velocidade Constante [m/s] (Ex: 5.0): ") or 5.0)

        config = ConfigEnsaio(raio=r_input, relacao=rel_input, tempo=t_input, perfil=perfil, v_constante=v_const)

        v_opt, i_opt, r_opt, reg_pid = executar_fase_pid(config)
        
        reg_orig, en_orig = executar_malha_aberta(config, V_ORIGINAL, R_ORIGINAL, "FASE 2/3 parametros originais")
        reg_opt, en_opt = executar_malha_aberta(config, v_opt, r_opt, "FASE 3/3 parametros modificados")
        
        plotar_otimizador(reg_orig, reg_opt, reg_pid, en_orig, en_opt, v_opt, i_opt,V_ORIGINAL,I_ORIGINAL)

    except ValueError:
        print("\n[ERRO] Valor inserido invalido. A simulacao precisa de numeros nas configuracoes.")