#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

def main():
    print("Caricamento dei dati e calcolo del costo combinato...")
    try:
        df = pd.read_csv('reachability_results.csv')
    except FileNotFoundError:
        print("Errore: file 'reachability_results.csv' non trovato.")
        return

    df['Z_round'] = df['Z'].round(3)
    shelves_z = sorted(df['Z_round'].unique())
    
    # =================================================================
    # RICALCOLO DEL COSTO
    # =================================================================
    df['Cost'] = np.nan
    valid_mask = (df['Reachable'] == 1) & (df['Yoshikawa'] > 0)
    df.loc[valid_mask, 'Cost'] = df.loc[valid_mask, 'Distance'] / df.loc[valid_mask, 'Yoshikawa']
    
    # =================================================================
    # CALCOLO LIMITI GLOBALI PER LA COLORBAR UNICA
    # =================================================================
    if df['Cost'].notna().any():
        global_vmin = df['Cost'].min()
        # Usiamo il 95° percentile per evitare che singoli picchi (es. costo 300) 
        # rovinino il gradiente per le zone a costo normale (es. 20-50)
        global_vmax = df['Cost'].quantile(0.95)
    else:
        global_vmin, global_vmax = 0, 1 # Fallback se tutto è vuoto

    width = max(8, 6 * len(shelves_z))
    fig, axes = plt.subplots(1, len(shelves_z), figsize=(width, 6))
    if len(shelves_z) == 1:
        axes = [axes]

    # Mappa di colori globale
    cmap = plt.get_cmap('RdYlGn_r').copy()
    cmap.set_bad(color='#111111') 

    for i, z_val in enumerate(shelves_z):
        shelf_data = df[df['Z_round'] == z_val].copy()
        
        shelf_data['X_round'] = shelf_data['X'].round(2)
        shelf_data['Y_round'] = shelf_data['Y'].round(2)
        
        pivot_cost = shelf_data.pivot_table(
            index='X_round', columns='Y_round', values='Cost', aggfunc='min', dropna=False
        )
        pivot_reach = shelf_data.pivot_table(
            index='X_round', columns='Y_round', values='Reachable', aggfunc='max', dropna=False
        )
        
        annot_matrix = np.where((pivot_reach == 0) | (pivot_reach.isna()), 'X', '')
        
        ax = axes[i]
        
        # Disegniamo la heatmap spegnendo la cbar individuale (cbar=False) 
        # e forzando i limiti globali (vmin, vmax)
        sns.heatmap(pivot_cost, ax=ax, cmap=cmap, cbar=False, 
                    vmin=global_vmin, vmax=global_vmax,
                    linewidths=0.5, linecolor='gray', square=True,
                    annot=annot_matrix, fmt='', 
                    annot_kws={'color': 'white', 'weight': 'bold', 'size': 12})
        
        ax.set_title(f'Ripiano Z = {z_val} m', fontsize=12)
        ax.set_xlabel('Y (Larghezza) [m]')
        ax.set_ylabel('X (Profondità) [m]')
        ax.invert_yaxis()

    plt.suptitle('TIAGo Pro: Ottimizzazione Place ', fontsize=16, fontweight='bold')
    
    # =================================================================
    # CREAZIONE DELLA COLORBAR UNICA A DESTRA
    # =================================================================
    # Stringiamo i grafici per lasciare l'8% di spazio vuoto a destra
    plt.tight_layout(rect=[0, 0, 0.92, 1]) 
    
    # Aggiungiamo un asse dedicato per la barra: [left, bottom, width, height]
    cbar_ax = fig.add_axes([0.93, 0.15, 0.02, 0.7])
    
    # Creiamo un "mappabile" fittizio per disegnare la legenda
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=global_vmin, vmax=global_vmax))
    sm.set_array([])
    fig.colorbar(sm, cax=cbar_ax, label='Costo (Minore = Migliore)')
    
    print("Generazione grafico in corso... Chiudi la finestra per terminare.")
    plt.show()

if __name__ == '__main__':
    main()