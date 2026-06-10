import numpy as np

class TurbinaVirtual:    
    def __init__(self,
                 constante_velocidade,
                 tau_armadura,
                 raio_turbina,
                 inercia_turbina,
                 coeficiente_atrito_turbina,
                 taxa_variacao_rpm_maxima,
                 velocidade_angular_maxima_pitch,
                 offset_fem = 0.0,
                 tsr_ideal = 7.5,
                 ganho_armadura_fixo = None,
                 torque_maximo_motor = None,
                 tabela_ganho_armadura = None,
                 capacitancia_barramento = 0.0,
                 relacao_transmissao_caixa_engrenagens = 1.0,
                 inercia_gerador = 0.0,
                 coeficiente_atrito_gerador = 0.0,
                 corrente_armadura_inicial = 0.0,
                 velocidade_angular_turbina_inicial = 0.0,
                 densidade_ar_kg_m3 = 1.2754,
                 modelo_coeficiente_potencia = 'classico', # classico/simplificado
                 sinal_controle_mppt = 'resistencia de carga'): # resistencia de carga/corrente de carga
        """
        Inicializa o modelo da turbina eólica.

        Args:
            modelo_coeficiente_potencia (str): Opções aceitas: 'classico' ou 'simplificado'.
            sinal_controle_mppt (str): Opções aceitas: 'resistencia de carga' ou 'corrente de carga'.
        """
        
        self._flag_vazio = True
        self._flag_torque_estatico = True
        self._modelo_coeficiente_potencia = modelo_coeficiente_potencia
        self._sinal_controle_mppt = sinal_controle_mppt
        self._torque_maximo_motor_nm = torque_maximo_motor
        
        self._constante_velocidade_gerador = constante_velocidade
        self._offset_fem = offset_fem

        self._ganho_armadura_fixo = ganho_armadura_fixo
        if self._ganho_armadura_fixo is not None:
            self._ganho_armadura = float(ganho_armadura_fixo)
        else:
            if tabela_ganho_armadura is None:
                raise ValueError("Você deve fornecer uma tabela de ganho se o ganho não for fixo.")
            self._keys_lookup_ganho_arm = np.array(sorted(tabela_ganho_armadura.keys()), dtype=float)
            self._values_lookup_ganho_arm = np.array([tabela_ganho_armadura[k] for k in self._keys_lookup_ganho_arm], dtype=float)
            self._ganho_armadura = self._values_lookup_ganho_arm[0]
        
        self._tau_armadura = tau_armadura
        self._capacitancia_farads = capacitancia_barramento

        self._relacao_transmissao_caixa_engrenagens = relacao_transmissao_caixa_engrenagens
        self._densidade_ar_kg_m3 = densidade_ar_kg_m3 
        self._raio_turbina_metros = raio_turbina
        self._area_varrida_turbina_m2 = np.pi * raio_turbina**2
        self._tsr_ideal = tsr_ideal

        self._inercia_turbina = inercia_turbina
        self._inercia_gerador = inercia_gerador
        self._inverso_inercia_equivalente_sistema = 1 / (inercia_gerador + (inercia_turbina / (relacao_transmissao_caixa_engrenagens**2)))

        self._coeficiente_atrito_turbina = coeficiente_atrito_turbina
        self._coeficiente_atrito_gerador = coeficiente_atrito_gerador
        self._coeficiente_atrito_equivalente = coeficiente_atrito_gerador + (coeficiente_atrito_turbina / (relacao_transmissao_caixa_engrenagens**2))

        self.taxa_variacao_rpm_maxima = taxa_variacao_rpm_maxima
        self._taxa_variacao_angulo_pas_rad_s = np.deg2rad(velocidade_angular_maxima_pitch)        

        self._corrente_armadura_amperes = corrente_armadura_inicial
        self._tensao_capacitor_volts = offset_fem
        self._velocidade_angular_turbina_rad_s = velocidade_angular_turbina_inicial
        self._velocidade_angular_gerador_rad_s = velocidade_angular_turbina_inicial * relacao_transmissao_caixa_engrenagens
        
        self._forca_eletromotriz = 0.0
        self._angulo_pas_radianos = 0.0
        self._angulo_pas_anterior_radianos = 0.0

        self._potencia_perdas_efeito_joule_w = 0.0
        self._potencia_perdas_atrito_pas_w = 0.0
        self._potencia_perdas_atrito_gerador_w = 0.0
        self._potencia_absorvida_vento_w = 0.0
        self._potencia_eixo_alta_velocidade_w = 0.0
        self._potencia_gerada_w = 0.0
        self._potencia_inercial_w = 0.0
        self._potencia_eletrica_entregue_w = 0.0
        
        self._torque_eletromagnetico_gerador_nm = 0.0
        self._torque_aerodinamico_no_gerador_nm = 0.0
        self._torque_atrito_pas_nm = 0.0
        self._torque_atrito_gerador_nm = 0.0
        self._torque_aceleracao_nm = 0.0
        
        self._erro_balanco_energetico_w = 0.0

    def set_flag_vazio(self, valor): self._flag_vazio = valor

    def get_flag_vazio(self):                       return self._flag_vazio
    def get_flag_torque_estatico(self):             return self._flag_torque_estatico

    def get_velocidade_angular_turbina_rad_s(self): return self._velocidade_angular_turbina_rad_s
    def get_velocidade_angular_gerador_rad_s(self): return self._velocidade_angular_gerador_rad_s

    def get_corrente_armadura_amperes(self):        return self._corrente_armadura_amperes
    def get_tensao_capacitor_volts(self):           return self._tensao_capacitor_volts
    
    def get_forca_eletromotriz(self):               return self._forca_eletromotriz
    def get_angulo_pas_radianos(self):              return self._angulo_pas_radianos
    
    def get_potencia_absorvida_vento_w(self):       return self._potencia_absorvida_vento_w
    def get_potencia_gerada_w(self):                return self._potencia_gerada_w
    def get_potencia_eletrica_entregue_w(self):     return self._potencia_eletrica_entregue_w
    def get_potencia_inercial_w(self):              return self._potencia_inercial_w
    def get_potencia_perdas_atrito_pas_w(self):     return self._potencia_perdas_atrito_pas_w
    def get_potencia_perdas_atrito_gerador_w(self): return self._potencia_perdas_atrito_gerador_w
    def get_potencia_perdas_efeito_joule_w(self):   return self._potencia_perdas_efeito_joule_w

    def get_torque_aerodinamico_no_gerador_nm(self): return self._torque_aerodinamico_no_gerador_nm
    def get_torque_eletromagnetico_gerador_nm(self): return self._torque_eletromagnetico_gerador_nm
    def get_torque_aceleracao_nm(self):              return self._torque_aceleracao_nm
    def get_torque_atrito_pas_nm(self):              return self._torque_atrito_pas_nm
    def get_torque_atrito_gerador_nm(self):          return self._torque_atrito_gerador_nm

    def get_erro_balanco_energetico_w(self):         return self._erro_balanco_energetico_w


    def atualizar_ganho_armadura(self):
        if self._ganho_armadura_fixo is not None:
            self._ganho_armadura = self._ganho_armadura_fixo
        else:
            velocidade_rpm = self.get_velocidade_angular_gerador_rad_s() * (60.0 / (2.0 * np.pi))
            self._ganho_armadura = np.interp(velocidade_rpm, self._keys_lookup_ganho_arm, self._values_lookup_ganho_arm)
        
        self._ganho_armadura = max(1e-4, self._ganho_armadura)
        
    def calcular_coeficiente_potencia(self, tip_speed_ratio, angulo_pas_radianos):
        if self._modelo_coeficiente_potencia == 'classico':
            angulo_pas_graus = np.rad2deg(angulo_pas_radianos)
            constante_1, constante_2, constante_3, constante_5, constante_6 = 0.5, 116.0, 0.4, 5.0, 21.0
            
            denominador_termo_1 = tip_speed_ratio + 0.08 * angulo_pas_graus
            termo_1 = 1.0 / denominador_termo_1 if denominador_termo_1 != 0 else 0.0
            termo_2 = 0.035 / (angulo_pas_graus**3 + 1.0)
            
            lambda_inverso = termo_1 - termo_2
            coeficiente = constante_1 * (constante_2 * lambda_inverso - constante_3 * angulo_pas_graus - constante_5) * np.exp(-constante_6 * lambda_inverso)
            
        if self._modelo_coeficiente_potencia == 'simplificado':
            fator_seno = np.sin(((np.pi * tip_speed_ratio) / 15.0 - 0.3 * angulo_pas_radianos))
            coeficiente = ((0.44 - 0.167 * angulo_pas_radianos) * fator_seno) - 0.16 * tip_speed_ratio * angulo_pas_radianos
            
        return max(0.0, coeficiente)
    
    def calcular_coeficiente_torque(self,velocidade_angular_turbina_rad_s,tip_speed_ratio, angulo_pas_radianos):
        LAMBDA_MINIMO = 6.75
        CQ_ESTATICO_PARTIDA = 0.07

        if tip_speed_ratio <= LAMBDA_MINIMO and velocidade_angular_turbina_rad_s <= 5 and self._flag_vazio:
            self._flag_torque_estatico = True
            cp_transicao = self.calcular_coeficiente_potencia(LAMBDA_MINIMO, angulo_pas_radianos)
            cq_transicao = cp_transicao / LAMBDA_MINIMO
            alfa = tip_speed_ratio / LAMBDA_MINIMO
            return CQ_ESTATICO_PARTIDA * (1.0 - alfa) + cq_transicao * alfa 
        else:
            self._flag_torque_estatico = False
            cp = self.calcular_coeficiente_potencia(tip_speed_ratio, angulo_pas_radianos)
            if tip_speed_ratio == 0.0:
                return 0.0
            return cp / tip_speed_ratio

    def calcular_torque_aerodinamico_no_gerador_nm(self, velocidade_vento_m_s, velocidade_angular_turbina_rad_s, angulo_pas_radianos):
        torque_maximo_permitido = self._torque_maximo_motor_nm if self._torque_maximo_motor_nm is not None else 1e100

        if velocidade_vento_m_s <= 0.0: 
            return 0.0
            
        tip_speed_ratio = (self._raio_turbina_metros * velocidade_angular_turbina_rad_s) / velocidade_vento_m_s
        
        cq = self.calcular_coeficiente_torque(velocidade_angular_turbina_rad_s,tip_speed_ratio,angulo_pas_radianos)
        
        torque_calculado_na_turbina = (0.5 * self._densidade_ar_kg_m3 * self._area_varrida_turbina_m2 * self._raio_turbina_metros * (velocidade_vento_m_s**2) * cq)
        torque_calculado_no_gerador = torque_calculado_na_turbina / self._relacao_transmissao_caixa_engrenagens

        return min(torque_calculado_no_gerador, torque_maximo_permitido)
      

    def calcular_derivada_tensao_capacitor(self, corrente_armadura, tensao_capacitor_atual, resistencia_carga_ohms = 0, corrente_carga = 0):
        if self._capacitancia_farads <= 0.0:
            return 0.0
            
        corrente_armadura = max(0.0, corrente_armadura)
        tensao_capacitor_atual = max(0.0, tensao_capacitor_atual)
        
        if self._sinal_controle_mppt == 'resistencia de carga':
            resistencia_efetiva = max(0.01, resistencia_carga_ohms) 
            corrente_carga = tensao_capacitor_atual / resistencia_efetiva
        if self._sinal_controle_mppt == 'corrente de carga':
            corrente_carga = corrente_carga
            
        corrente_fuga = 0.01
        corrente_liquida = corrente_armadura - corrente_carga - corrente_fuga
        return corrente_liquida / self._capacitancia_farads

    def calcular_derivada_corrente_armadura_no_gerador(self, corrente_atual, velocidade_gerador_atual, tensao_capacitor_atual):
        corrente_atual = max(0.0, corrente_atual)
        tensao_capacitor_atual = max(0.0, tensao_capacitor_atual)
        velocidade_gerador_atual = max(0.0, velocidade_gerador_atual)
        
        if velocidade_gerador_atual>0.0:
            forca_eletromotriz = (self._constante_velocidade_gerador * velocidade_gerador_atual) + self._offset_fem
        else:
            forca_eletromotriz = 0.0

        derivada = ((self._ganho_armadura * (forca_eletromotriz - tensao_capacitor_atual)) - corrente_atual) / self._tau_armadura
        
        return derivada

    def calcular_derivada_velocidade_angular_no_gerador(self, corrente_atual, velocidade_gerador_atual, velocidade_vento_m_s, angulo_pas_radianos):        
        corrente_atual = max(0.0, corrente_atual)
        velocidade_gerador_atual = max(0.0, velocidade_gerador_atual)
        
        torque_aerodinamico_no_gerador = self.calcular_torque_aerodinamico_no_gerador_nm(velocidade_vento_m_s, velocidade_gerador_atual / self._relacao_transmissao_caixa_engrenagens, angulo_pas_radianos)
        torque_eletromagnetico_gerador = (self._constante_velocidade_gerador * corrente_atual)
        torque_friccao = self._coeficiente_atrito_equivalente * velocidade_gerador_atual
        
        aceleracao_gerador_teorica = self._inverso_inercia_equivalente_sistema * (
                    torque_aerodinamico_no_gerador - torque_eletromagnetico_gerador - torque_friccao)
        
        limite_aceleracao_gerador_rad_s2 = self.taxa_variacao_rpm_maxima * (2.0 * np.pi / 60.0) 

        return np.clip(aceleracao_gerador_teorica, -limite_aceleracao_gerador_rad_s2, limite_aceleracao_gerador_rad_s2)

    def executar_passo_simulacao(self, velocidade_vento_m_s, angulo_pas_alvo_radianos, passo_tempo_segundos, resistencia_carga_ohms = 0, corrente_carga = 0, delta_ganho_armadura = 0):

        self.atualizar_ganho_armadura()
        self._ganho_armadura += delta_ganho_armadura
        self._ganho_armadura = max(1e-4, self._ganho_armadura)
        
        variacao_maxima_angulo = self._taxa_variacao_angulo_pas_rad_s * passo_tempo_segundos
        diferenca_angulo = angulo_pas_alvo_radianos - self._angulo_pas_anterior_radianos
        self._angulo_pas_radianos = self._angulo_pas_anterior_radianos + np.clip(diferenca_angulo, -variacao_maxima_angulo, variacao_maxima_angulo)
        self._angulo_pas_anterior_radianos = self._angulo_pas_radianos
        
        # Passo 1 (k1)
        k1_corrente = passo_tempo_segundos * self.calcular_derivada_corrente_armadura_no_gerador(self._corrente_armadura_amperes, self._velocidade_angular_gerador_rad_s, self._tensao_capacitor_volts)
        k1_velocidade = passo_tempo_segundos * self.calcular_derivada_velocidade_angular_no_gerador(self._corrente_armadura_amperes, self._velocidade_angular_gerador_rad_s, velocidade_vento_m_s,self._angulo_pas_radianos)
        k1_tensao_cap = passo_tempo_segundos * self.calcular_derivada_tensao_capacitor(self._corrente_armadura_amperes, self._tensao_capacitor_volts, resistencia_carga_ohms=resistencia_carga_ohms, corrente_carga=corrente_carga)

        # Passo 2 (k2)
        k2_corrente = passo_tempo_segundos * self.calcular_derivada_corrente_armadura_no_gerador(self._corrente_armadura_amperes + 0.5 * k1_corrente, self._velocidade_angular_gerador_rad_s + 0.5 * k1_velocidade, self._tensao_capacitor_volts + 0.5 * k1_tensao_cap)
        k2_velocidade = passo_tempo_segundos * self.calcular_derivada_velocidade_angular_no_gerador(self._corrente_armadura_amperes + 0.5 * k1_corrente, self._velocidade_angular_gerador_rad_s + 0.5 * k1_velocidade, velocidade_vento_m_s,self._angulo_pas_radianos)
        k2_tensao_cap = passo_tempo_segundos * self.calcular_derivada_tensao_capacitor(self._corrente_armadura_amperes + 0.5 * k1_corrente, self._tensao_capacitor_volts + 0.5 * k1_tensao_cap,  resistencia_carga_ohms=resistencia_carga_ohms, corrente_carga=corrente_carga)

        # Passo 3 (k3)
        k3_corrente = passo_tempo_segundos * self.calcular_derivada_corrente_armadura_no_gerador(self._corrente_armadura_amperes + 0.5 * k2_corrente, self._velocidade_angular_gerador_rad_s + 0.5 * k2_velocidade, self._tensao_capacitor_volts + 0.5 * k2_tensao_cap)
        k3_velocidade = passo_tempo_segundos * self.calcular_derivada_velocidade_angular_no_gerador(self._corrente_armadura_amperes + 0.5 * k2_corrente, self._velocidade_angular_gerador_rad_s + 0.5 * k2_velocidade, velocidade_vento_m_s,self._angulo_pas_radianos)
        k3_tensao_cap = passo_tempo_segundos * self.calcular_derivada_tensao_capacitor(self._corrente_armadura_amperes + 0.5 * k2_corrente, self._tensao_capacitor_volts + 0.5 * k2_tensao_cap,  resistencia_carga_ohms=resistencia_carga_ohms, corrente_carga=corrente_carga)

        # Passo 4 (k4)
        k4_corrente = passo_tempo_segundos * self.calcular_derivada_corrente_armadura_no_gerador(self._corrente_armadura_amperes + k3_corrente, self._velocidade_angular_gerador_rad_s + k3_velocidade, self._tensao_capacitor_volts + k3_tensao_cap)
        k4_velocidade = passo_tempo_segundos * self.calcular_derivada_velocidade_angular_no_gerador(self._corrente_armadura_amperes + k3_corrente, self._velocidade_angular_gerador_rad_s + k3_velocidade, velocidade_vento_m_s,self._angulo_pas_radianos)
        k4_tensao_cap = passo_tempo_segundos * self.calcular_derivada_tensao_capacitor(self._corrente_armadura_amperes + k3_corrente, self._tensao_capacitor_volts + k3_tensao_cap,  resistencia_carga_ohms=resistencia_carga_ohms, corrente_carga=corrente_carga)

        self._corrente_armadura_amperes += (k1_corrente + 2.0 * k2_corrente + 2.0 * k3_corrente + k4_corrente) / 6.0
        self._velocidade_angular_gerador_rad_s += (k1_velocidade + 2.0 * k2_velocidade + 2.0 * k3_velocidade + k4_velocidade) / 6.0
        self._tensao_capacitor_volts += (k1_tensao_cap + 2.0 * k2_tensao_cap + 2.0 * k3_tensao_cap + k4_tensao_cap) / 6.0

        self._corrente_armadura_amperes = max(0.0, self._corrente_armadura_amperes)
        self._velocidade_angular_gerador_rad_s = max(0.0, self._velocidade_angular_gerador_rad_s)
        self._tensao_capacitor_volts = max(0.0, self._tensao_capacitor_volts)
        
        self._velocidade_angular_turbina_rad_s = self._velocidade_angular_gerador_rad_s / self._relacao_transmissao_caixa_engrenagens

        self._forca_eletromotriz = (self._constante_velocidade_gerador * self._velocidade_angular_gerador_rad_s) + self._offset_fem

        self._torque_aerodinamico_no_gerador_nm = self.calcular_torque_aerodinamico_no_gerador_nm(velocidade_vento_m_s,self._velocidade_angular_turbina_rad_s,self._angulo_pas_radianos)
        self._torque_eletromagnetico_gerador_nm = self._constante_velocidade_gerador * self._corrente_armadura_amperes
        self._torque_atrito_pas_nm = self._coeficiente_atrito_turbina * self._velocidade_angular_turbina_rad_s
        self._torque_atrito_gerador_nm = self._coeficiente_atrito_gerador * self._velocidade_angular_gerador_rad_s
        
        self._torque_aceleracao_nm = (self._torque_aerodinamico_no_gerador_nm - 
                                      (self._torque_atrito_pas_nm / self._relacao_transmissao_caixa_engrenagens) - 
                                      self._torque_atrito_gerador_nm - 
                                      self._torque_eletromagnetico_gerador_nm)

        self._potencia_absorvida_vento_w = self._torque_aerodinamico_no_gerador_nm * self._velocidade_angular_gerador_rad_s
        self._potencia_inercial_w = self._torque_aceleracao_nm * self._velocidade_angular_gerador_rad_s
        self._potencia_gerada_w = self._torque_eletromagnetico_gerador_nm * self._velocidade_angular_gerador_rad_s 
        self._potencia_eletrica_entregue_w = self._tensao_capacitor_volts * self._corrente_armadura_amperes

        resistencia_equivalente = 1.0 / self._ganho_armadura
        indutancia_equivalente = self._tau_armadura / self._ganho_armadura
        
        self._potencia_perdas_efeito_joule_w = resistencia_equivalente * (self._corrente_armadura_amperes ** 2)
        self._potencia_perdas_atrito_pas_w = (self._torque_atrito_pas_nm / self._relacao_transmissao_caixa_engrenagens) * self._velocidade_angular_gerador_rad_s
        self._potencia_perdas_atrito_gerador_w = self._torque_atrito_gerador_nm * self._velocidade_angular_gerador_rad_s        

        derivada_corrente_instantanea = self.calcular_derivada_corrente_armadura_no_gerador(
            self._corrente_armadura_amperes, 
            self._velocidade_angular_gerador_rad_s, 
            self._tensao_capacitor_volts
        )
        
        self._potencia_dinamica_indutancia_w = indutancia_equivalente * self._corrente_armadura_amperes * derivada_corrente_instantanea

        potencia_entrada_total_w = self._potencia_absorvida_vento_w + self._offset_fem * self._corrente_armadura_amperes
        
        potencia_saida_e_armazenada_w = (self._potencia_perdas_atrito_pas_w + 
                                         self._potencia_perdas_atrito_gerador_w + 
                                         self._potencia_inercial_w + 
                                         self._potencia_perdas_efeito_joule_w + 
                                         self._potencia_dinamica_indutancia_w + 
                                         self._potencia_eletrica_entregue_w)

        self._erro_balanco_energetico_w = potencia_entrada_total_w - potencia_saida_e_armazenada_w