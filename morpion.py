from pyniryo import *
from minimax import *

def detect_objects_positions(robot, coordCases, game, depth = 0.32):
    img_compressed = robot.get_img_compressed()
    img = uncompress_image(img_compressed)
    img_threshold = threshold_hsv(img, *ColorHSV.ANY.value)
    # Appliquer des transformations morphologiques (par exemple, ouverture) pour améliorer l'image
    img_threshold = morphological_transformations(img_threshold, morpho_type=MorphoType.OPEN,
                                                  kernel_shape=(11, 11), kernel_type=KernelType.ELLIPSE)
    # Trouver les contours dans l'image seuillée
    cnts = biggest_contours_finder(img_threshold, 10)  # Trouver les 5 plus gros contours
    
    x_origin = 195
    y_origin = 305

    for cnt in cnts:
        # Trouver le barycentre de chaque contour
        cnt_barycenter = get_contour_barycenter(cnt)
        cx, cy = cnt_barycenter
        if 180 < cx < 420 and 70 < cy < 300 :
            # Afficher la position du barycentre (x, y) de l'objet
            #print(f"Objet détecté à la position (x: {cx - x_origin}, y: {y_origin - cy})")

            # Obtenir les paramètres de la caméra
            mtx, dist = robot.get_camera_intrinsics()

            # Paramètres de calibration de la caméra
            fx = mtx[0, 0]
            fy = mtx[1, 1]

            # Conversion pixels → mètres avec les focales
            x_real = (cx - x_origin) * depth / fx
            y_real = (y_origin - cy) * depth / fy
            
            closest_case = get_closest_case([x_real, y_real], coordCases)
            shape = detect_shape(cnt)
            game[closest_case] = shape

            # Pour visualisation, dessiner le contour et le barycentre
            img_debug = draw_contours(img_threshold, [cnt])
            img_debug = draw_barycenter(img_debug, cx, cy)

            # Afficher l'image de débogage
            #show_img_and_wait_close("Image avec barycentre", img_debug)

    return game
    

def move_to_object(robot, object_position, z_position=0.1, depth=0.32):
    """
    Déplace le robot à la position de l'objet détecté à partir d'une image.
    
    :param robot: instance de NiryoRobot
    :param object_position: tuple (x, y) de la position en pixels dans l'image
    :param z_position: hauteur cible pour le robot
    :param depth: estimation de la profondeur (mètres)
    """
    # Obtenir les paramètres de la caméra
    mtx, dist = robot.get_camera_intrinsics()

    # Paramètres de calibration de la caméra
    fx = mtx[0, 0]
    fy = mtx[1, 1]

    # Conversion pixels → mètres avec les focales
    x_real = object_position[0] * depth / fx
    y_real = object_position[1] * depth / fy
    z_real = z_position

    print(f"Objet détecté à {object_position} (px)")
    print(f"Coordonnées réelles estimées : x={x_real:.3f} m, y={y_real:.3f} m, z={z_real:.3f} m")

    # Créer une pose robot et se déplacer
    pose = PoseObject(x=x_real, y=y_real, z=z_real, roll=0.0, pitch=1.57, yaw=0.0)
    robot.move_pose(pose, "Morpion")
    
def get_closest_case(position, cases):
    """
    Trouve l'index de la case la plus proche de la position donnée.
    :param position: tuple (x, y) - coordonnées de l'objet
    :param cases: liste des positions des centres des cases
    :return: index de la case la plus proche
    """
    min_distance = float('inf')
    closest_index = -1
    for i, case in enumerate(cases):
        dx = position[0] - case[0]
        dy = position[1] - case[1]
        distance = (dx**2 + dy**2)**0.5
        if distance < min_distance:
            min_distance = distance
            closest_index = i
    return closest_index
    
def detect_shape(cnt):
    """
    Détecte la forme (cercle ou carré) d'un contour.
    :param cnt: contour
    :return: int - 1 si 'cercle' ou 2 si 'carré'
    """
    approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
    if len(approx) == 4:
        return 2
    else:
        return 1
        
def check_winner(game):
    lines = [
        [0, 1, 2],  # ligne 1
        [3, 4, 5],  # ligne 2
        [6, 7, 8],  # ligne 3
        [0, 3, 6],  # colonne 1
        [1, 4, 7],  # colonne 2
        [2, 5, 8],  # colonne 3
        [0, 4, 8],  # diagonale 1
        [2, 4, 6],  # diagonale 2
    ]
    
    for line in lines:
        if game[line[0]] == game[line[1]] == game[line[2]] and game[line[0]] != 0:
            if game[line[0]] == 1:  # Retourne le gagnant (0 ou 1)
                print("Le joueur '1' (cercle) a gagné.")
            else:
                print("Le joueur '0' (carré) a gagné.")
            return False
                
    # Vérification d'égalité (si toutes les cases sont remplies et qu'il n'y a pas de gagnant)
    if all(x != 0 for x in game):
        print("La partie est terminée par égalité.")
        return False
    
    # La partie continue si aucune condition n'est remplie
    print("La partie continue.")
    return True

def play(case, pionPose, observationJoints, coup):
    robot.move_pose(pionPose)
    robot.close_gripper(400)

    robot.move_joints(observationJoints)
    robot.move_pose(PoseObject(case[coup[0]*3 + coup[1]][0], case[coup[0]*3 + coup[1]][1], 0.09, 0, 1.57, 0), "Morpion")
    robot.open_gripper(400)

    robot.move_joints(observationJoints)
    
    
if __name__ == '__main__':
    robot = NiryoRobot("10.10.10.10")
    robot.calibrate_auto()
    
    robot.set_brightness(0.9)
    robot.set_contrast(1.0)
    robot.set_saturation(1.1)
    
    #if "Morpion" not in robot.get_workspace_list():
    #    workspacePoints = [[0.3813, 0.0843, 0.0281],
    #                    [0.3759, -0.0870, 0.0274],
    #                    [0.2025, -0.0837, 0.0192],
    #                    [0.2090, 0.0874, 0.0192]]
    #    robot.save_workspace_from_points("Morpion", workspacePoints[0], workspacePoints[1], workspacePoints[2], workspacePoints[3])
    
    origin = [0.288, 0, 0.05]
    x_axe = [0.288, -0.10, 0.05]
    y_axe = [0.35, 0, 0.05]
    #robot.save_dynamic_frame_from_points("Morpion", "Un repère dynamique prenant comme origine le centre du plateau de jeu", origin, x_axe, y_axe, True)
    
    origin2 = [0.2029, 0.0868, 0.0188]
    x_axe2 = [0.2004, -0.0858, 0.0190]
    y_axe2 = [0.3718, 0.0836,0.0272]
    robot.save_dynamic_frame_from_points("Morpion", "Un repère dynamique prenant comme origine le repère en bas en gauche du workspace", origin2, x_axe2, y_axe2)
    
    #enregistre la position relative en x et y des cases dans le repère dynamique Morpion en partant de haut à gauche et en finissant en bas à droite
    caseMorpion = [[0.025, 0.155],
                    [0.09, 0.155],
                    [0.155, 0.155],
                    [0.025, 0.09],
                    [0.09, 0.09],
                    [0.155, 0.09],
                    [0.025, 0.025],
                    [0.09, 0.025],
                    [0.155, 0.025]]
    
    homeJoints = [0, 0.5, -1.25, 0, 0, 0]
    observationJoints = [0, 0.22, -0.4, 0, -1.6, 0] 

    pickPose = PoseObject(0.2768, -0.1265, 0.1091, 0.177, 1.251, -1.431)
    
    #Donne l'état actuel du jeu, -1 si pas de pion, 0 pour un carré et 1 pour un cercle
    game = [0 for i in range(9)]
    newGame = game.copy()
    
    playing = True
    
    #robot.move_joints(homeJoints)
    #robot.open_gripper(400)
    
    #robot.move_joints(observationJoints)
    #robot.move_pose(pickPoses[0])
    #robot.move_pose(pickPoses[1])
    #robot.close_gripper(400)
    #robot.move_pose(pickPoses[0])

    #robot.move_joints(observationJoints)

    #robot.move_pose(PoseObject(caseMorpion[0][0], caseMorpion[0][1], 0.09, 0, 1.57, 0), "Morpion")
    #robot.open_gripper(400)
    
    #if object_positions:
    #    for i in range(len(object_positions)):
    #        move_to_object(robot, object_positions[i])
    #        robot.wait(2)
    
    #robot.move_pose(PoseObject(0, 0, 0.2, 0, 1.57, 0), "Morpion")
    #robot.move_pose(PoseObject(0, 0.1, 0, 0, 1.57, 0), "Morpion")
    #robot.move_pose(PoseObject(0.1, 0, 0, 0, 1.57, 0), "Morpion")
    
    #robot.move_joints(homeJoints)
    
    #robot.delete_workspace("Morpion")

    while playing :
        change = False
        while not change:
            detect_objects_positions(robot, caseMorpion, newGame)
            if newGame != game:
                change = True
                game = newGame.copy()
            print("waiting")
            robot.wait(3)
        print("Etat du jeu :", newGame)
        playing = check_winner(game)
        #trouve prochain coups --> à faire
        #check_winner(game, playing)
    
    robot.close_connection()
