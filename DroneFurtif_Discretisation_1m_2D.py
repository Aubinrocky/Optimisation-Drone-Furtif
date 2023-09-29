import os
import time
import random
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
# On definit le relief, qui comporte d'une part la liste des points du relief, et la longueur du relief.

# liste des 8 déplacements possibles pris par la trajectoire à t+dt:
listeDeplacementsPossibles = [[1, 1], [1, 0], [1, -1],
                              [2, 2], [2, 1], [2, 0], [2, -1], [2, -2]]
evolutionFinesse = []


# Ensuite, il faut pouvoir gérer les reliefs. On doit pouvoir :
# - Creer un relief
# - Creer un relief au hasard
# - Lire et stocker un relief a partir d'un fichier .txt

class gestionnaireRelief:
    # on stocke le relief choisit dans la variable suivante :
    def __init__(self):
        self.reliefChoisi = []

    # On cree la fonction qui nous permettra de lire un relief a partir d'un fichier .txt
    def lireRelief(self, fichierALire):
        fichier = open(fichierALire, "r")    # On ouvre le fichier dans Python
        longueurRelief = 0  # on initialise le nb de points du relief
        for ligne in fichier:  # on parcourt le fichier ligne par ligne
            if longueurRelief == 0:  # si on parcourt la première ligne, on ne stocke pas les données vu que ce sont les titres des colonnes
                # on sépare l'abscisse et l'ordonnée qui sont séparés par un espace sur la meme ligne dans le texte
                point = ligne.split(' ')
                # on ajoute 1 au relief pour pouvoir parcourir le fichier, que l'on retirera a la fin
                longueurRelief += 1
            else:
                point = ligne.split(' ')
                # on convertit la chaine de caractère en nombre entier
                point[0] = int(point[0])
                # idem mais il faut retirer le terme "\n" dont on ne veut pas avant de pouvoir convertir en nombre flottant
                pointProv = point[1].split('\n')
                # on convertit la chaine de caractere en nb flottant
                point[1] = float(pointProv[0])
                # on ajoute le point a la liste de points du relief
                self.reliefChoisi.append(point)
                longueurRelief += 1  # on a lu un point de plus
        fichier.close()

    # On cree une fonction qui calcule la longueur du relief
    def longueurRelief(self):
        return len(self.reliefChoisi)

    # on crée une fonction qui permet d'obtenir la latitude d'un point connaissant son indice longitudinal :
    def getLatitude(self, indice):
        return self.reliefChoisi[indice][1]


# on definit ensuite la classe trajectoire.
# La trajectoire est caractérisée par :
# - La liste des points de la trajectoire
# - la liste des déplacements de la trajectoire
# - La vitesse moy du drone ayant cette trajectoire
# - le temps de mission du drone ayant cette trajectoire
# - l'erreur sur l'altitude du drone ayant cette trajectoire
# - la finesse qui se déduit des trois critères précédents

class trajectoire:

    def __init__(self, GestionnaireRelief, trajectoire=None):
        self.GestionnaireRelief = GestionnaireRelief
        self.trajectoire = []
        self.deplacements = []
        self.distanceParcourue = 0.0
        self.vitesseMoyenne = 0.0
        self.tempsMission = 0.0
        self.erreurAltitude = 0.0
        self.vraieErreurAltitude = 0.0
        self.ponderationDistanceParcourue = 0.33
        self.ponderationTempsMission = 0.33
        self.ponderationErreurAltitude = 0.33
        self.finesse = 0.0
        self.pointInitial = [0, 3010]
        self.pointFinal = [len(GestionnaireRelief.reliefChoisi), 3010]
        self.distanceParcourueOptimale = 0.0
        self.tempsMissionOptimal = 0.0
        self.erreurAltitudeOptimale = 2
        self.hauteurSecurite = 2
        self.distancePointFinal = self.GestionnaireRelief.longueurRelief()
        self.longTot = self.pointFinal[0] - self.pointInitial[0]

        # on initialise les trajectoires :
        if trajectoire is not None:
            self.trajectoire = trajectoire
        else:
            self.trajectoire.append(self.pointInitial)
            for i in range(1, self.GestionnaireRelief.longueurRelief()):
                self.trajectoire.append(None)

    def __len__(self):
        return len(self.trajectoire)

    def __getItem__(self, index):
        return self.trajectoire[index]

    def __setItem__(self, key, value):
        self.trajectoire[key] = value

    # on definit la fonction qui va nous permettre de générer une trajectoire au hasard :
    def genererIndividu(self):
        self.genererDeplacements()
        self.miseAJourTrajectoire(self.deplacements)

    def genererDeplacements(self):
        indicePositionX = 0
        # le drone doit etre au depart assez haut par rapport au relief :
        if self.trajectoire[0][1] >= (self.GestionnaireRelief.reliefChoisi[0][1] + self.hauteurSecurite):
            while indicePositionX <= self.GestionnaireRelief.longueurRelief()-2:
                if indicePositionX == self.GestionnaireRelief.longueurRelief()-2:
                    self.deplacements.append(
                        random.choice([[1, 0], [1, -1], [1, 1]]))
                    indicePositionX += 1
                else:
                    # on choisit un déplacement au hasard pour le drone :
                    deplacementChoisi = random.choice(
                        listeDeplacementsPossibles)
                    # on l'ajoute dans la liste des deplacements
                    self.deplacements.append(deplacementChoisi)

                    # Différents cas possibles :
                    # cas où on se déplace que d'un rang horizontalement :
                    if deplacementChoisi[0] == 1:
                        # dans ce cas, la position n'evolue que de 1 :
                        indicePositionX += 1
                    # cas où on se déplace de deux rangs horizontalement :
                    if deplacementChoisi[0] == 2:
                        # on avance de deux dans la boucle:
                        indicePositionX += 2

    def miseAJourTrajectoire(self, deplacements):
        k = 0
        indicePositionX = 1
        self.deplacements = deplacements
        while k < len(deplacements):
            # on ajoutait cette cond pour le cone d'arrivée : and (self.trajectoire[indicePositionX-1][1]<self.pointFinal[1]+ self.longTot - indicePositionX) and (self.trajectoire[indicePositionX-1][1]>self.pointFinal[1] - (self.longTot-indicePositionX))
            if deplacements[k][0] == 1:
                self.trajectoire[indicePositionX] = [self.trajectoire[indicePositionX - 1]
                                                     [0] + 1, self.trajectoire[indicePositionX - 1][1] + deplacements[k][1]]

                # si le drone se crash : on change de deplacement jusqu'à relever assez le drone pour qu'il ne se crash pas :
                # limite = 0
                # while abs(self.trajectoire[indicePositionX][1]-self.GestionnaireRelief.reliefChoisi[indicePositionX][1])<10 and limite<20:
                #     self.deplacements[k] = (random.choice([[1,0],[1,-1],[1,1]]))
                #     self.trajectoire[indicePositionX] = [ self.trajectoire[indicePositionX - 1][0] + 1 , self.trajectoire[indicePositionX - 1][1] + deplacements[k][1] ]
                #     limite +=1
                if abs(self.trajectoire[indicePositionX][1]-self.GestionnaireRelief.reliefChoisi[indicePositionX][1]) < self.hauteurSecurite:
                    self.deplacements[k] = [1, 1]
                    self.trajectoire[indicePositionX] = [
                        self.trajectoire[indicePositionX - 1][0] + 1, self.trajectoire[indicePositionX - 1][1] + 1]
                    # si on touche tjs le relief, le drone ne peut plus esquiver le relief, on le crash :
                    if abs(self.trajectoire[indicePositionX][1]-self.GestionnaireRelief.reliefChoisi[indicePositionX][1]) < self.hauteurSecurite:
                        k = len(deplacements)

                # on avance d'un rang :
                indicePositionX += 1
            else:
                self.trajectoire[indicePositionX] = [self.trajectoire[indicePositionX-1]
                                                     [0] + 1, self.trajectoire[indicePositionX-1][1] + deplacements[k][1]/2]
                self.trajectoire[indicePositionX+1] = [self.trajectoire[indicePositionX-1]
                                                       [0] + 2, self.trajectoire[indicePositionX-1][1] + deplacements[k][1]]

                if abs(self.trajectoire[indicePositionX][1]-self.GestionnaireRelief.reliefChoisi[indicePositionX][1]) < self.hauteurSecurite or abs(self.trajectoire[indicePositionX+1][1]-self.GestionnaireRelief.reliefChoisi[indicePositionX+1][1]) < self.hauteurSecurite:
                    self.deplacements[k] = [2, 2]
                    self.trajectoire[indicePositionX] = [
                        self.trajectoire[indicePositionX - 1][0] + 1, self.trajectoire[indicePositionX - 1][1] + 1]
                    self.trajectoire[indicePositionX+1] = [self.trajectoire[indicePositionX-1]
                                                           [0] + 2, self.trajectoire[indicePositionX-1][1] + 2]
                    # si on touche tjs le relief, le drone ne peut plus esquiver le relief, on le crash :
                    if abs(self.trajectoire[indicePositionX][1]-self.GestionnaireRelief.reliefChoisi[indicePositionX][1]) < self.hauteurSecurite or abs(self.trajectoire[indicePositionX+1][1]-self.GestionnaireRelief.reliefChoisi[indicePositionX+1][1]) < self.hauteurSecurite:
                        k = len(deplacements)
                # on avance ensuite de 2 rangs :
                indicePositionX += 2

            k += 1

    def getPoint(self, indice):
        return self.trajectoire[indice]

    def setPoint(self, indice, point):
        self.trajectoire[indice] = point

        # si on rajoute un point, il faut remettre les compteurs a zero
        self.finesse = 0.0
        self.vitesseMoyenne = 0.0
        self.tempsMission = 0.0
        self.erreurAltitude = 0.0

    # on s'occupe ensuite de determiner et pouvoir obtenir les differents criteres de notation du vol :
    def calculDistanceParcourue(self):
        somme = 0
        i = 0
        while (self.trajectoire[i] != None) and (i < len(self.deplacements)):
            somme += math.sqrt(self.deplacements[i]
                               [0]**2+self.deplacements[i][1]**2)
            i += 1
        self.distanceParcourue = somme

    def calculVitesseMoyenne(self):
        # on met dabord a jour la distance et tps pendant la mission :
        self.calculDistanceParcourue()
        self.calculTempsMission()
        # on calcule ensuite la nouvelle vitesse moyenne :
        self.vitesseMoyenne = 3.6*self.distanceParcourue / self.tempsMission

    def calculTempsMission(self):
        # on a un pas de 0.1 a chaque deplacement effectué
        self.tempsMission = 0.05 * len(self.deplacements)

    def calculErreurAltitude(self):
        err = 0
        i = 0
        err2 = 0
        while (i < self.GestionnaireRelief.longueurRelief()) and (self.trajectoire[i] != None):
            err += (self.trajectoire[i][1] -
                    self.GestionnaireRelief.reliefChoisi[i][1])**2
            err2 += abs(self.trajectoire[i][1] -
                        self.GestionnaireRelief.reliefChoisi[i][1])
            i += 1
        self.erreurAltitude = err/self.GestionnaireRelief.longueurRelief()
        self.vraieErreurAltitude = err2/self.GestionnaireRelief.longueurRelief()

    def calculDistanceParcourueOptimale(self):
        self.distanceParcourueOptimale = math.sqrt(
            (self.pointFinal[1]-self.pointInitial[1])**2+(self.pointFinal[0]-self.pointInitial[0])**2)

    def calculTempsMissionOptimal(self):
        self.tempsMissionOptimal = 0.05 * self.tailleTrajectoire()

    def calculDistancePointFinal(self):
        self.distancePointFinal = math.sqrt((self.pointFinal[1]-self.trajectoire[self.tailleTrajectoire(
        )-1][1])**2+(self.pointFinal[0]-self.trajectoire[self.tailleTrajectoire()-1][0])**2)

    def calculFinesse(self):
        # on met a jour les differentes valeurs :
        self.calculDistanceParcourue()
        self.calculTempsMission()
        self.calculErreurAltitude()
        self.calculDistanceParcourueOptimale()
        self.calculTempsMissionOptimal()
        self.calculDistancePointFinal()

        if self.tailleTrajectoire() == self.longTot:
            self.finesse = self.erreurAltitudeOptimale/self.erreurAltitude
        else:
            self.finesse = 0

    def getVitesseMoyenne(self):
        self.calculVitesseMoyenne()
        return self.vitesseMoyenne

    def getTempsMission(self):
        self.calculTempsMission()
        return self.tempsMission

    def getErreurAltitude(self):
        self.calculErreurAltitude()
        return self.vraieErreurAltitude

    def getFinesse(self):
        self.calculFinesse()
        return self.finesse

    def tailleTrajectoire(self):
        i = 0
        while i < len(self.trajectoire) and self.trajectoire[i] != None:
            i += 1

        return i

    def contientPoint(self, Point):
        return Point in self.trajectoire

    def getDistancePointFinal(self):
        self.calculDistancePointFinal()
        return self.distancePointFinal


# dans l'algorihtme genetique, on travaille sur des populations,
# càd des ensembles de plusieurs trajectoires.
# On definit donc la classe de population qui nous servira par la suite.

class population:
    # on initialise la population
    def __init__(self, GestionnaireRelief, taillePopulation, init):
        self.trajectoires = []

        # on initialise la liste des trajcetoires par une liste de taille  souhaitée (taille population) de cases vides
        for i in range(taillePopulation):
            self.trajectoires.append(None)

        # dans le cas où le booleen init est true, on definit les nouvelles trajectoires,
        # générées au hasard grace aux fonctions def precedemment dans la classe trajectoires.
        if init:
            for i in range(0, taillePopulation):
                nouvelleTrajectoire = trajectoire(GestionnaireRelief)
                nouvelleTrajectoire.genererIndividu()
                self.sauvegarderTrajectoire(i, nouvelleTrajectoire)

    def __setitem__(self, key, value):
        self.trajectoires[key] = value

    def __getitem__(self, index):
        return self.trajectoires[index]

    # fonction qui permet d'enregistrer une trajectoire précise dans la population :
    def sauvegarderTrajectoire(self, indice, trajectoire):
        self.trajectoires[indice] = trajectoire

    # fonction qui permet d'obtenir une trajectoire parmis la population :
    def getTrajectoire(self, indice):
        return self.trajectoires[indice]

    # fonction qui permet de récupérer la trajectoire la meilleure trajectoire :
    def getMeilleureTrajectoire(self):
        meilleureTrajectoire = self.trajectoires[0]
        for i in range(0, self.taillePopulation()):
            if meilleureTrajectoire.getFinesse() <= self.getTrajectoire(i).getFinesse():
                meilleureTrajectoire = self.getTrajectoire(i)

        return meilleureTrajectoire

    # fonction qui permet d'obtenir la taille de la population de trajectoires
    def taillePopulation(self):
        return len(self.trajectoires)


class GA:
    def __init__(self, GestionnaireRelief):
        self.GestionnaireRelief = GestionnaireRelief

        # def du taux de mutation = proba qu'un point d'une trajectoire subisse une mutation :
        self.tauxMutation = 0.015
        # selection choisie : par tournoi (pls individus st selec au hasard, et nous gardons l'individu ayant la meilleure finesse.):
        self.tailleTournoi = 5
        # elitisme = vouloir conserver les meilleurs individus d'une génération à l'autre, pour etre sur de pas les perdre --> accelere la cvg de l'algo au détriment de la diversité des individus. Dans notre cas, le nb d'individus conservés est égal à 1 (on garde que le meilleur).
        self.elitisme = True

    def evoluerPopulation(self, pop):
        # on cree une generation de population de trajectoires
        nouvellePopulation = population(
            self.GestionnaireRelief, pop.taillePopulation(), False)

        # elitismeOffset = 1 si on a conservé la valeur elitiste, = 0 si c'est pas encore fait
        elitismeOffset = 0
        # dans le cas ou nous sommes elitistes, on conserve la meilleure trajectoire de la génération précédentes, et on la stocke dans la premiere case de la liste nouvellePopulation :
        if self.elitisme:
            nouvellePopulation.sauvegarderTrajectoire(
                0, pop.getMeilleureTrajectoire())
            elitismeOffset = 1

        # on réalise le cross over dans la population : on met l'enfant créé par deux parents, dans chq case de liste de la nouvelle generation :
        # si l'algo est elitiste, on ne commence qu'à l'indice 1 car la meilleure trajectoire de la génération précédente est déjà conservée dans la 0ème case
        for i in range(elitismeOffset, nouvellePopulation.taillePopulation()):
            parent1 = self.selectionParRang(pop)  # un 1er parent est choisi
            parent2 = self.selectionParTournoi(
                pop)  # un 2eme parent est choisi
            # un enfant est genere a partir des deux parents
            enfant = self.crossover(parent1, parent2)
            # on sauvegarde l'enfant dans la nvlle génération
            nouvellePopulation.sauvegarderTrajectoire(i, enfant)

        # A chaque trajectoire de la nouvelle génération obtenue (exceptée pour la trajectoire optimale conservée case 0), on lui fait subir une mutation génétique :
        for i in range(elitismeOffset, nouvellePopulation.taillePopulation()):
            nouvellePopulation.trajectoires[i] = self.muter(
                nouvellePopulation.trajectoires[i])
        evolutionFinesse.append(nouvellePopulation[0].getFinesse())
        return nouvellePopulation

    def crossover(self, parent1, parent2):
        # on initialise un enfant qui sera le resultat du crossover :
        enfant = trajectoire(self.GestionnaireRelief)
        # on choisit dans un premier temps un simple crossover :
        # On prend un abscisse au hasard pour créer la reproduction :
        indiceCrossoverTrajectoire = int(
            random.random() * parent1.GestionnaireRelief.longueurRelief())

        # on mixe fait donc un mix des parents pour creer l'enfant :
        for i in range(0, indiceCrossoverTrajectoire):
            enfant.setPoint(i, parent1.getPoint(i))

        for i in range(indiceCrossoverTrajectoire, parent1.GestionnaireRelief.longueurRelief()):
            enfant.setPoint(i, parent2.getPoint(i))

        indiceCrossoverDeplacementsParent1 = 0
        i = 0
        # on fait ensuite tourner une boucle pour récupérer l'indice du dernier déplacement de parent1 pour arriver à indiceCrossoverTrajectoire:
        while i < indiceCrossoverTrajectoire and indiceCrossoverDeplacementsParent1 < len(parent1.deplacements):
            i += parent1.deplacements[indiceCrossoverDeplacementsParent1][0]
            indiceCrossoverDeplacementsParent1 += 1
        indiceCrossoverDeplacementsParent1 -= 1
        # on fait la meme chose pour obtenir l'indice du dernier déplacement avant d'arriver à l'indiceCrosseoverTrajectoire mais pour le parent2:
        indiceCrossoverDeplacementsParent2 = 0
        i = 0
        while i < indiceCrossoverTrajectoire and indiceCrossoverDeplacementsParent2 < len(parent2.deplacements):
            i += parent2.deplacements[indiceCrossoverDeplacementsParent2][0]
            indiceCrossoverDeplacementsParent2 += 1
        indiceCrossoverDeplacementsParent2 -= 1
        # Ensuite, on ajoute les déplacements faits dans parent1 entre 0 et indiceCrossoverTrajectoire :
        for k in range(0, indiceCrossoverDeplacementsParent1):
            enfant.deplacements.append(parent1.deplacements[k])
        for k in range(indiceCrossoverDeplacementsParent2+1, len(parent2.deplacements)):
            enfant.deplacements.append(parent2.deplacements[k])

        enfant.miseAJourTrajectoire(enfant.deplacements)
        return enfant

    def muter(self, trajectoire):
        # voici notre mutation : pour chaque deplacement effectué :
        # si le hasard nous dit qu'on mute ce deplacement :
        # si ce deplacement est de 1m horizontalement, alors il devient un deplacement au hasard possible d'abscisse 1 m
        # si ce deplacement est de 2m horizontalement, alors il devient un deplacement au hasard possible d'abscisse 2 m

        for indiceDeplacement in range(0, len(trajectoire.deplacements)):
            if random.random() < self.tauxMutation:
                if trajectoire.deplacements[indiceDeplacement][0] == 1:
                    trajectoire.deplacements[indiceDeplacement] = random.choice(
                        [[1, -1], [1, 0], [1, 1]])
                else:
                    trajectoire.deplacements[indiceDeplacement] = random.choice(
                        [[2, -1], [2, 0], [2, 1], [2, -2], [2, 2]])

        for indiceDeplacement in range(0, len(trajectoire.deplacements)):
            if random.random() < self.tauxMutation:
                if trajectoire.deplacements[indiceDeplacement][0] == 1:
                    trajectoire.deplacements[indiceDeplacement] = random.choice(
                        [[1, -1], [1, 0], [1, 1]])
                else:
                    trajectoire.deplacements[indiceDeplacement] = random.choice(
                        [[2, -1], [2, 0], [2, 1], [2, -2], [2, 2]])

        for indiceDeplacement in range(0, len(trajectoire.deplacements)):
            if random.random() < self.tauxMutation:
                if trajectoire.deplacements[indiceDeplacement][0] == 1:
                    trajectoire.deplacements[indiceDeplacement] = random.choice(
                        [[1, -1], [1, 0], [1, 1]])
                else:
                    trajectoire.deplacements[indiceDeplacement] = random.choice(
                        [[2, -1], [2, 0], [2, 1], [2, -2], [2, 2]])
        trajectoire.miseAJourTrajectoire(trajectoire.deplacements)
        return(trajectoire)

    # Fonction qui permet de trouver la meilleure trajectoire (plus fine) lors d'un tournoi :
    def selectionParTournoi(self, pop):
        # on initialise le tournoi qui va stocker la sous-population selectionnée :
        tournoi = population(self.GestionnaireRelief,
                             self.tailleTournoi, False)

        # on choisit chaque "membre" du tournoi par hasard dans la population actuelle :
        for i in range(0, self.tailleTournoi):
            # on determine l'indice de la trajectoire choisie au hasard
            indiceRandomTrajectoire = int(
                random.random() * pop.taillePopulation())
            tournoi.sauvegarderTrajectoire(i, pop.getTrajectoire(
                indiceRandomTrajectoire))  # on sauvegarde cette trajectoire
        meilleureTrajectoire = tournoi.getMeilleureTrajectoire()

        return meilleureTrajectoire

    def selectionParRang(self, pop):

        # On somme toutes les finesses de trajectoires de la population :
        sommeFinesse = 0
        for i in range(0, pop.taillePopulation()):
            sommeFinesse += pop.trajectoires[i].getFinesse()

        # on choisit un nombre au hasard entre 0 et la somme des finesses :
        choix = random.uniform(0, sommeFinesse)
        # On choisit la trajectoire dans la population : plus la finesse d'une traj est grande, plus cette traj a de chances d'être selectionée
        i = 0
        sommeFinesse = 0
        while i < pop.taillePopulation()-1 and choix > sommeFinesse:
            sommeFinesse += pop.trajectoires[i].finesse
            i += 1

        return pop.trajectoires[i]

    def iterationEvolution(self, nbGenerations, taillePop):
        # initialisation
        finesses = []
        erreurs = []
        vitessesMoy = []
        tempss = []
        pop = population(self.GestionnaireRelief, taillePop, True)
        # evolution des generations
        for k in range(nbGenerations):
            pop = self.evoluerPopulation(pop)
            finesses.append(pop.getMeilleureTrajectoire().getFinesse())
            erreurs.append(pop.getMeilleureTrajectoire().getErreurAltitude())
            tempss.append(pop.getMeilleureTrajectoire().getTempsMission())
            vitessesMoy.append(
                pop.getMeilleureTrajectoire().getVitesseMoyenne())
        return(pop, finesses, erreurs, vitessesMoy, tempss)


if __name__ == '__main__':

    # on crée un gestionnaire de reliefs :
    GR = gestionnaireRelief()

    # On implémente le relief stocké dans le fichier txt dans notre gestionnaire relief :
    GR.lireRelief('C:\\reliefalire.txt')  # verifier si ca fonctionne
    print(GR.longueurRelief())

    # on fait evoluer notre population sur 100 generations :
    ga = GA(GR)  # on creer une structure d'algo genetique

    # On fait evoluer une population de 100 trajectoires sur 100 generations :
    pop, finesses, erreurs, vitessesMoy, tempss = ga.iterationEvolution(
        100, 100)
    print(finesses)

    # on stocke la meilleure trajectoire
    meilleureTrajectoire = pop.getMeilleureTrajectoire()

    # Construction des listes qui vont afficher les reliefs et trajectoires :
    longsRelief = []
    longsTrajectoire = []
    latsRelief = []
    latsMeilleureTrajectoire = []
    for i in range(0, GR.longueurRelief()):
        longsRelief.append(i)
        # on prend les ordonnées des points du relief :
        latsRelief.append(GR.getLatitude(i))
    k = 0
    while k < GR.longueurRelief() and meilleureTrajectoire.getPoint(k) != None:
        longsTrajectoire.append(k)
        # on prend les ordonnées des points de la meilleure trajectoire :
        latsMeilleureTrajectoire.append(meilleureTrajectoire.getPoint(k)[1])
        k = k+1

    generations = [k for k in range(100)]

    # On lisse la courbe grâce au module scipy.signal :
    latsMeilleureTrajectoireLisse = savgol_filter(
        latsMeilleureTrajectoire, 51, 5)
    fig, axs = plt.subplots(4, 1)
    axs[0].plot(generations, erreurs, 'tab:orange')
    axs[0].set_title('Erreurs d altitude')
    axs[0].set_xlabel('Numéro itération')
    axs[0].set_ylabel('Erreurs d altitude')
    axs[1].plot(generations, vitessesMoy, 'tab:green')
    axs[1].set_title('Vitesses moyennes')
    axs[1].set_xlabel('Numéro itération')
    axs[1].set_ylabel('Vitesses moyennes')
    axs[2].plot(generations, tempss, 'tab:red')
    axs[2].set_title('Temps mission')
    axs[2].set_xlabel('Nb itérations')
    axs[2].set_ylabel('Temps mission')
    axs[3].plot(longsRelief, latsRelief, 'tab:blue')
    axs[3].plot(longsTrajectoire, latsMeilleureTrajectoire, 'tab:orange')

    axs[3].plot(longsTrajectoire, latsMeilleureTrajectoireLisse, 'tab:red')
    axs[3].set_title('MeilleureTrajectoire')
    axs[3].set_xlabel('Longitude')
    axs[3].set_ylabel('Latitude')

    for ax in axs.flat:
        ax.label_outer()
    plt.show()
