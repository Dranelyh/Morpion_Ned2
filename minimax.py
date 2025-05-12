# Dictionnaire des scores
scores = {
    True: 0,   # Égalité
    1: -1,     # Score pour le joueur : -1 (score à minimiser)
    2: 1,      # Score pour l'IA : 1 (score à maximiser)
}

def meilleur_coup(grille):
    meilleur_score = float('-inf')
    coup = None

    # Parcours de la grille pour chercher les cases vides
    for i in range(3):
        for j in range(3):
            if grille[i][j] == 0:
                grille[i][j] = 2  # 2 représente l'IA
                score = minimax(grille, False)
                grille[i][j] = 0  # Annuler le coup
                
                if score > meilleur_score:
                    meilleur_score = score
                    coup = (i, j)
    return coup

def minimax(grille, maximisation):
    resultat = gagner(grille)
    if resultat is not None:
        return scores[resultat]
    elif egalite(grille):
        return scores[True]
    
    if maximisation:
        meilleur_score = float('-inf')
        for i in range(3):
            for j in range(3):
                if grille[i][j] == 0:
                    grille[i][j] = 2  # 2 représente l'IA
                    score = minimax(grille, False)
                    grille[i][j] = 0  # Annuler le coup
                    meilleur_score = max(score, meilleur_score)
        return meilleur_score
    else:
        meilleur_score = float('inf')
        for i in range(3):
            for j in range(3):
                if grille[i][j] == 0:
                    grille[i][j] = 1  # 1 représente le joueur
                    score = minimax(grille, True)
                    grille[i][j] = 0  # Annuler le coup
                    meilleur_score = min(score, meilleur_score)
        return meilleur_score

# Exemple de fonctions de vérification de victoire et d'égalité (à adapter selon ton code existant)
def gagner(grille):
    # Logique pour vérifier si un joueur a gagné
    for i in range(3):
        # Lignes et colonnes
        if grille[i][0] == grille[i][1] == grille[i][2] != 0:
            return grille[i][0]
        if grille[0][i] == grille[1][i] == grille[2][i] != 0:
            return grille[0][i]
    # Diagonales
    if grille[0][0] == grille[1][1] == grille[2][2] != 0:
        return grille[0][0]
    if grille[0][2] == grille[1][1] == grille[2][0] != 0:
        return grille[0][2]
    return None

def egalite(grille):
    for i in range(3):
        for j in range(3):
            if grille[i][j] == 0:
                return False

