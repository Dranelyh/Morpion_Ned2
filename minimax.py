# Dictionnaire des scores (bruts)
scores = {
    True: 0,   # Égalité
    1: -1,     # Score pour le joueur (humain)
    2: 1,      # Score pour l'IA
}

def meilleur_coup(grille):
    meilleur_score = float('-inf')
    coup = None

    for i in range(3):
        for j in range(3):
            if grille[i][j] == 0:
                grille[i][j] = 2  # IA joue
                score = minimax(grille, False, profondeur=1)
                grille[i][j] = 0  # Annuler le coup

                if score > meilleur_score:
                    meilleur_score = score
                    coup = (i, j)
    return coup

def minimax(grille, maximisation, profondeur):
    resultat = gagner(grille)
    if resultat is not None:
        return (scores[resultat] * 10) - profondeur if resultat == 2 else (scores[resultat] * 10) + profondeur
    elif egalite(grille):
        return 0

    if maximisation:
        meilleur_score = float('-inf')
        for i in range(3):
            for j in range(3):
                if grille[i][j] == 0:
                    grille[i][j] = 2
                    score = minimax(grille, False, profondeur + 1)
                    grille[i][j] = 0
                    meilleur_score = max(score, meilleur_score)
        return meilleur_score
    else:
        meilleur_score = float('inf')
        for i in range(3):
            for j in range(3):
                if grille[i][j] == 0:
                    grille[i][j] = 1
                    score = minimax(grille, True, profondeur + 1)
                    grille[i][j] = 0
                    meilleur_score = min(score, meilleur_score)
        return meilleur_score

def gagner(grille):
    for i in range(3):
        if grille[i][0] == grille[i][1] == grille[i][2] != 0:
            return grille[i][0]
        if grille[0][i] == grille[1][i] == grille[2][i] != 0:
            return grille[0][i]
    if grille[0][0] == grille[1][1] == grille[2][2] != 0:
        return grille[0][0]
    if grille[0][2] == grille[1][1] == grille[2][0] != 0:
        return grille[0][2]
    return None

def egalite(grille):
    return all(grille[i][j] != 0 for i in range(3) for j in range(3))