scores = {
    True: 0,   # égalité
    1: -1,     # score pour l'utilisateur
    2: 1,      # score pour le robot
}

def meilleur_coup(grille):
    """
    Détermine le meilleur coup que le robot peut jouer à partir de l'état actuel de la grille.

    Le robot simule tous les coups possibles en appelant l'algorithme Minimax et choisit celui qui donne le meilleur score.

    Args:
        grille (list[list[int]]): La grille de jeu, une matrice 3x3 avec 0 pour vide, 1 pour l'adversaire, et 2 pour le robot.

    Returns:
        tuple[int, int]: Les coordonnées (ligne, colonne) du meilleur coup trouvé.
    """
    meilleur_score = float('-inf')
    coup = None

    for i in range(3):
        for j in range(3):
            if grille[i][j] == 0:
                grille[i][j] = 2  # le robot teste toutes les possibilités
                score = minimax(grille, False, profondeur=1) 
                grille[i][j] = 0  # annule le coup

                if score > meilleur_score:
                    meilleur_score = score # garde le meilleur coup
                    coup = (i, j)
    return coup

def minimax(grille, maximisation, profondeur):
    """
    Applique l'algorithme Minimax pour évaluer le score d'une grille de morpion.

    Args:
        grille (list[list[int]]): La grille de jeu, une matrice 3x3 avec 0 pour vide, 1 pour l'adversaire, et 2 pour le robot.
        maximisation (bool): True si c'est au robot de jouer, False si c'est à l'adversaire.
        profondeur (int): La profondeur actuelle dans l'arbre de décision (sert à ajuster les scores pour privilégier les victoires rapides).

    Returns:
        int: Le score évalué de la position actuelle.
    """
    resultat = gagner(grille) # on teste si la partie est finie
    if resultat is not None: 
        return (scores[resultat] * 10) - profondeur if resultat == 2 else (scores[resultat] * 10) + profondeur # plus la profondeur est grande, plus le score va être petit
    elif egalite(grille):
        return 0

    if maximisation: # c'est au robot de jouer
        meilleur_score = float('-inf')
        for i in range(3):
            for j in range(3):
                if grille[i][j] == 0:
                    grille[i][j] = 2
                    score = minimax(grille, False, profondeur + 1)
                    grille[i][j] = 0
                    meilleur_score = max(score, meilleur_score)
        return meilleur_score
    
    else: # c'est à l'adversaire du robot de jouer 
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
    """
    Vérifie si un joueur a gagné la partie sur la grille actuelle.

    La fonction teste toutes les lignes, colonnes et diagonales pour détecter une séquence de trois symboles identiques non nuls.

    Args:
        grille (list[list[int]]): La grille de jeu, une matrice 3x3 avec 0 pour vide, 1 pour l'adversaire, et 2 pour le robot.

    Returns:
        int | None: 1 si l'adversaire a gagné, 2 si le robot a gagné, None si aucun joueur n'a encore gagné.
    """
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
    """
    Vérifie si la partie est terminée par égalité (aucune case vide et aucun gagnant).

    Args:
        grille (list[list[int]]): La grille de jeu, une matrice 3x3 avec 0 pour vide, 1 pour l'adversaire, et 2 pour le robot.

    Returns:
        bool: True si la grille est pleine et qu'aucun joueur n'a gagné, False sinon.
    """
    return all(grille[i][j] != 0 for i in range(3) for j in range(3))