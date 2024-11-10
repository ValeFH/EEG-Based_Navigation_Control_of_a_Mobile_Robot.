# -*- coding: utf-8 -*-
"""AI_without_differentiating_mixed_train_initial_balanced_data.ipynb

Automatically generated by Colab.

Original file is located at
    https://colab.research.google.com/drive/1bnBjcxxrsCLy773GkR8TluZwTrURxymR
"""

# Commented out IPython magic to ensure Python compatibility.
'''
----------------------------------------------------------------------------------------------------------------------
LIBRERÍAS:
'''

#LIBRERIAS
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import matplotlib.patches as mpatches
import seaborn as sb

# %matplotlib inline
plt.rcParams['figure.figsize'] = (14, 7)
plt.style.use('default')

from scipy.spatial import distance
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split

from pandas.io.formats.style import Subset
from sklearn.decomposition import PCA, KernelPCA
from sklearn import decomposition
from sklearn.svm import SVC
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import LabelEncoder
from sklearn.neural_network import MLPClassifier
from sklearn.utils.validation import column_or_1d

from sklearn import svm

#NAIVE BAYES Y RANDOM FOREST
from sklearn.naive_bayes import GaussianNB
from sklearn.ensemble import RandomForestClassifier

#CROSS VALIDATION
from sklearn.model_selection import cross_val_score, cross_validate, cross_val_predict, KFold

#ANN
from sklearn.model_selection import GridSearchCV
from sklearn.neural_network import MLPClassifier

#Logistic Regression
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import PolynomialFeatures

#METRICAS
from sklearn.metrics import matthews_corrcoef
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix
from sklearn.metrics import f1_score
from sklearn.metrics import classification_report

from imblearn.over_sampling import RandomOverSampler
from imblearn.under_sampling import RandomUnderSampler

# Cantidad total de índices (número de voluntarios)
indices = list(range(1, 31))

'''
----------------------------------------------------------------------------------------------------------------------
DEFINICIÓN DE FUNCIONES
'''

'''
Función matriz de confusión:
Es utilizada para graficar la matriz de confusión en el formato deseado.
'''

'''
Función matriz de confusión:
Es utilizada para graficar la matriz de confusión en el formato deseado.
'''
#FUNCION MATRIZ DE CONFUSION
def matriz_confusion(y_test, y_pred, titulo='Matriz de Confusión', figsize=(8, 6), cmap="Blues"):
    # Calcular la matriz de confusión
    cm = confusion_matrix(y_test, y_pred)

    # Crear el mapa de calor
    plt.figure(figsize=figsize)
    sb.heatmap(cm, annot=True, fmt="d", cmap=cmap, cbar=False)

    # Etiquetas y título
    plt.title(titulo)
    plt.xlabel('Predicción')
    plt.ylabel('Actual')
    plt.show()

'''
Función métricas totales:
Genera las 3 métricas elegidas según el valor de y y y_pred.
'''
#FUNCIÓN MÉTRICAS TOTALES
def metricas(y, y_pred):
  # Calcular métricas
  f1 = f1_score(y, y_pred, average='weighted')
  print("F1 score:", f1)
  ACC = accuracy_score(y, y_pred)
  print("Accuracy", ACC)
  matthews = matthews_corrcoef(y, y_pred)
  print("Coeficiente de Matthews:", matthews)
  print(matriz_confusion(y, y_pred, titulo='Matriz de Confusión', figsize=(8, 6), cmap="Blues"))
  print(classification_report(y, y_pred))

  return [f1, ACC, matthews]

#FUNCIÓN MÉTRICAS POR ETIQUETA
def metricas_por_etiqueta(y_test, y_pred, label):
  # Calcular métricas
  f1 = f1_score(y_test, y_pred, labels=[label], average='weighted')
  acc = accuracy_score(y_test == label, y_pred == label)
  matthews = matthews_corrcoef(y_test, y_pred)
  print("F1 score:", f1)
  print("Accuracy", acc)
  print("Coeficiente de Matthews:", matthews)

  return [f1, acc, matthews]

#FUNCION PROMEDIO Y DESVIACION
def prom_desv(a_scores):
  promedio_f1 = np.mean(a_scores[:, 0])
  promedio_acc = np.mean(a_scores[:, 1])
  promedio_matthews = np.mean(a_scores[:, 2])
  desviacion_f1 = np.std(a_scores[:, 0])
  desviacion_acc = np.std(a_scores[:, 1])
  desviacion_matthews = np.std(a_scores[:, 2])

  return promedio_f1, promedio_acc, promedio_matthews, desviacion_f1, desviacion_acc, desviacion_matthews

# -------------------------
# NAIVE BAYES, RANDOM FOREST, SVM, REGRESIÓN LOGÍSTICA
# -------------------------

# Listas para almacenar las métricas
nb_scores, nb0_scores, nb1_scores, nb2_scores, nb3_scores = [[] for _ in range(5)]
rf_scores, rf0_scores, rf1_scores, rf2_scores, rf3_scores = [[] for _ in range(5)]
svm_scores, svm0_scores, svm1_scores, svm2_scores, svm3_scores = [[] for _ in range(5)]
lr_scores, lr0_scores, lr1_scores, lr2_scores, lr3_scores = [[] for _ in range(5)]
mlp_scores, mlp0_scores, mlp1_scores, mlp2_scores, mlp3_scores = [[] for _ in range(5)]

nb_scores_cas1, nb0_scores_cas1, nb1_scores_cas1, nb2_scores_cas1, nb3_scores_cas1 = [[] for _ in range(5)]
rf_scores_cas1, rf0_scores_cas1, rf1_scores_cas1, rf2_scores_cas1, rf3_scores_cas1 = [[] for _ in range(5)]
svm_scores_cas1, svm0_scores_cas1, svm1_scores_cas1, svm2_scores_cas1, svm3_scores_cas1 = [[] for _ in range(5)]
lr_scores_cas1, lr0_scores_cas1, lr1_scores_cas1, lr2_scores_cas1, lr3_scores_cas1 = [[] for _ in range(5)]
mlp_scores_cas1, mlp0_scores_cas1, mlp1_scores_cas1, mlp2_scores_cas1, mlp3_scores_cas1 = [[] for _ in range(5)]

nb_scores_cas2, nb0_scores_cas2, nb1_scores_cas2, nb2_scores_cas2, nb3_scores_cas2 = [[] for _ in range(5)]
rf_scores_cas2, rf0_scores_cas2, rf1_scores_cas2, rf2_scores_cas2, rf3_scores_cas2 = [[] for _ in range(5)]
svm_scores_cas2, svm0_scores_cas2, svm1_scores_cas2, svm2_scores_cas2, svm3_scores_cas2 = [[] for _ in range(5)]
lr_scores_cas2, lr0_scores_cas2, lr1_scores_cas2, lr2_scores_cas2, lr3_scores_cas2 = [[] for _ in range(5)]
mlp_scores_cas2, mlp0_scores_cas2, mlp1_scores_cas2, mlp2_scores_cas2, mlp3_scores_cas2 = [[] for _ in range(5)]

nb_scores_cas3, nb0_scores_cas3, nb1_scores_cas3, nb2_scores_cas3, nb3_scores_cas3 = [[] for _ in range(5)]
rf_scores_cas3, rf0_scores_cas3, rf1_scores_cas3, rf2_scores_cas3, rf3_scores_cas3 = [[] for _ in range(5)]
svm_scores_cas3, svm0_scores_cas3, svm1_scores_cas3, svm2_scores_cas3, svm3_scores_cas3 = [[] for _ in range(5)]
lr_scores_cas3, lr0_scores_cas3, lr1_scores_cas3, lr2_scores_cas3, lr3_scores_cas3 = [[] for _ in range(5)]
mlp_scores_cas3, mlp0_scores_cas3, mlp1_scores_cas3, mlp2_scores_cas3, mlp3_scores_cas3 = [[] for _ in range(5)]

# Crear una instancia de StandardScaler
scaler = StandardScaler()

"""
Se extraen n archivos y se concatenan en un solo dataframe
"""
#dataframe = pd.read_csv(r"matriz_concatenada_1.csv",sep=',', skiprows=1)
#print(dataframe.shape)
#dataframe = pd.read_csv(r"matriz_concatenada_2.csv",sep=',', skiprows=1)
#print(dataframe.shape)

dataframes = []
dataframes1 = []
n = 30
# Leer y agregar cada DataFrame a la lista
for i in range(1, n + 1):
    archivo1 = f"imaginario_voluntario{i}.csv"
    archivo = f"motor_voluntario{i}.csv"

    #archivo = f"matriz_motor_{i}.csv"
    #archivo1 = f"matriz_imaginativo_{i}.csv"

    #archivo1 = f"matriz_motor_{i}.csv"
    #archivo = f"matriz_imaginativo_{i}.csv"

    df = pd.read_csv(archivo, sep=',', header=None)
    df1 = pd.read_csv(archivo1, sep=',', header=None)
    dataframes.append(df)
    dataframes1.append(df1)

# Concatenar todos los DataFrames por filas (axis=0)
dataframe = pd.concat(dataframes, axis=0, ignore_index=True)
dataframe1 = pd.concat(dataframes1, axis=0, ignore_index=True)

X1 = dataframe.iloc[:, :56].astype(float).values  # Selecciona las primeras 32 columnas
y1 = dataframe.iloc[:, 60].astype(float).values  # Selecciona y

X2 = dataframe1.iloc[:, :56].astype(float).values  # Selecciona las primeras 32 columnas
y2 = dataframe1.iloc[:, 60].astype(float).values  # Selecciona y

X3 = dataframe1.iloc[:, :56].astype(float).values  # Selecciona las primeras 32 columnas
y3 = dataframe1.iloc[:, 61].astype(float).values  # Selecciona y

print(X1.shape)
print(y1.shape)

oversampler = RandomOverSampler(random_state=42)
X1, y1 = oversampler.fit_resample(X1, y1)
X2, y2 = oversampler.fit_resample(X2, y2)
#X3, y3 = oversampler.fit_resample(X3, y3)

print(X1.shape)
print(y1.shape)

plt.figure()
plt.scatter(X1[:,0],X1[:,1],c=y1,alpha=0.8)
plt.show()

for i in range(3):
    X_train, X_test, y_train, y_test = train_test_split(X1, y1, test_size=0.2, random_state=i)
    X_train1, X_test1, y_train1, y_test1 = train_test_split(X2, y2, test_size=0.2, random_state=i)
    X_train2, X_test2, y_train2, y_test2 = train_test_split(X3, y3, test_size=0.2, random_state=i)

    #Se concatenan los datos de entrenamiento para el motor + imaginario dif y sin dif
    X_train = np.concatenate((X_train, X_train1), axis=0)
    y_train = np.concatenate((y_train, y_train1), axis=0)
    #X_train = np.concatenate((X_train, X_train2), axis=0)

    #Se concatenan motor e imaginario diferenciado y sin diferenciar
    X_test_sindif = np.concatenate((X_test, X_test1), axis=0)
    X_test_condif = np.concatenate((X_test, X_test2), axis=0)
    y_test_sindif = np.concatenate((y_test, y_test1), axis=0)
    y_test_condif = np.concatenate((y_test, y_test2), axis=0)

    y_test2=y_test_sindif
    y_test3=y_test_condif

    print("Train: ",X_train.shape, y_train.shape)
    print("Test: ",X_test.shape, y_test.shape)
    print("Test: ",X_test1.shape, y_test1.shape)
    print("Test sin dif: ",X_test_sindif.shape, y_test_sindif.shape)
    print("Test con dif: ",X_test_condif.shape, y_test_condif.shape)


    scaler.fit(X_train)
    X_train = scaler.transform(X_train)
    X_test = scaler.transform(X_test) # modelo entrenado
    X_test1 = scaler.transform(X_test1) # modelo opuesto al entrenado
    X_test_sindif = scaler.transform(X_test_sindif) # Ambos modelos sin dif
    X_test_condif = scaler.transform(X_test_condif) # Ambos modelos con dif

    gnb = GaussianNB()
    # Entrenar el modelo NAIVE BAYES
    gnb.fit(X_train, y_train)
    # Realizar predicciones en el conjunto de prueba
    y_pred = gnb.predict(X_test)
    y_pred1 = gnb.predict(X_test1)
    y_pred2 = gnb.predict(X_test_sindif)
    y_pred3 = gnb.predict(X_test_condif)

    print("MODELO NAIVE BAYES")
    # Métricas Naive Bayes
    nb_scores.append(metricas(y_test, y_pred))
    nb0_scores.append(metricas_por_etiqueta(y_test, y_pred, 0))
    nb1_scores.append(metricas_por_etiqueta(y_test, y_pred, 1))
    nb2_scores.append(metricas_por_etiqueta(y_test, y_pred, 2))
    nb3_scores.append(metricas_por_etiqueta(y_test, y_pred, 3))

    print("MODELO NAIVE BAYES 1")
    nb_scores_cas1.append(metricas(y_test1, y_pred1))
    print("MODELO NAIVE BAYES 2")
    nb_scores_cas2.append(metricas(y_test2, y_pred2))
    print("MODELO NAIVE BAYES 3")
    nb_scores_cas3.append(metricas(y_test3, y_pred3))

    # Random Forest
    rf_model = RandomForestClassifier(n_estimators=100)
    rf_model.fit(X_train, y_train)
    y_pred_rf = rf_model.predict(X_test)
    y_pred_rf1 = rf_model.predict(X_test1)
    y_pred_rf2 = rf_model.predict(X_test_sindif)
    y_pred_rf3 = rf_model.predict(X_test_condif)

    print("MODELO RANDOM FOREST")
    # Métricas Random Forest
    rf_scores.append(metricas(y_test, y_pred_rf))
    rf0_scores.append(metricas_por_etiqueta(y_test, y_pred_rf, 0))
    rf1_scores.append(metricas_por_etiqueta(y_test, y_pred_rf, 1))
    rf2_scores.append(metricas_por_etiqueta(y_test, y_pred_rf, 2))
    rf3_scores.append(metricas_por_etiqueta(y_test, y_pred_rf, 3))
    print("MODELO RANDOM FOREST 1")
    rf_scores_cas1.append(metricas(y_test1, y_pred_rf1))
    print("MODELO RANDOM FOREST 2")
    rf_scores_cas2.append(metricas(y_test2, y_pred_rf2))
    print("MODELO RANDOM FOREST 3")
    rf_scores_cas3.append(metricas(y_test3, y_pred_rf3))

    # SVM
    svm_model = svm.SVC(C=1000, kernel='poly', degree = 10, coef0=1, gamma = 'auto')
    svm_model.fit(X_train, y_train)
    y_pred_svm = svm_model.predict(X_test)
    y_pred_svm1 = svm_model.predict(X_test1)
    y_pred_svm2 = svm_model.predict(X_test_sindif)
    y_pred_svm3 = svm_model.predict(X_test_condif)

    print("MODELO SVM")

    # Métricas SVM
    svm_scores.append(metricas(y_test, y_pred_svm))
    svm0_scores.append(metricas_por_etiqueta(y_test, y_pred_svm, 0))
    svm1_scores.append(metricas_por_etiqueta(y_test, y_pred_svm, 1))
    svm2_scores.append(metricas_por_etiqueta(y_test, y_pred_svm, 2))
    svm3_scores.append(metricas_por_etiqueta(y_test, y_pred_svm, 3))

    print("MODELO SVM 1")
    svm_scores_cas1.append(metricas(y_test1, y_pred_svm1))
    print("MODELO SVM 2")
    svm_scores_cas2.append(metricas(y_test2, y_pred_svm2))
    print("MODELO SVM 3")
    svm_scores_cas3.append(metricas(y_test3, y_pred_svm3))

    # Regresión Logística
    lr_model = LogisticRegression(penalty='l2')
    lr_model.fit(X_train, y_train)
    y_pred_lr = lr_model.predict(X_test)
    y_pred_lr1 = lr_model.predict(X_test1)
    y_pred_lr2 = lr_model.predict(X_test_sindif)
    y_pred_lr3 = lr_model.predict(X_test_condif)

    print("MODELO REGRESIÓN LOGÍSTICA")

    # Métricas Regresión Logística
    lr_scores.append(metricas(y_test, y_pred_lr))
    lr0_scores.append(metricas_por_etiqueta(y_test, y_pred_lr, 0))
    lr1_scores.append(metricas_por_etiqueta(y_test, y_pred_lr, 1))
    lr2_scores.append(metricas_por_etiqueta(y_test, y_pred_lr, 2))
    lr3_scores.append(metricas_por_etiqueta(y_test, y_pred_lr, 3))

    lr_scores_cas1.append(metricas(y_test1, y_pred_lr1))
    lr_scores_cas2.append(metricas(y_test2, y_pred_lr2))
    lr_scores_cas3.append(metricas(y_test3, y_pred_lr3))

    #ANN
    mlp = MLPClassifier(hidden_layer_sizes=(100,90,80,70,60,50,40,30), activation='relu', learning_rate='adaptive', max_iter=300)
    # Entrenar el modelo ANN
    mlp.fit(X_train, y_train)
    # Realizar predicciones en el conjunto de prueba
    y_pred_mlp = mlp.predict(X_test)
    y_pred_mlp1 = mlp.predict(X_test1)
    y_pred_mlp2 = mlp.predict(X_test_sindif)
    y_pred_mlp3 = mlp.predict(X_test_condif)

    print("MODELO ANN")
    # Métricas ANN
    mlp_scores.append(metricas(y_test, y_pred_mlp))
    mlp0_scores.append(metricas_por_etiqueta(y_test, y_pred_mlp, 0))
    mlp1_scores.append(metricas_por_etiqueta(y_test, y_pred_mlp, 1))
    mlp2_scores.append(metricas_por_etiqueta(y_test, y_pred_mlp, 2))
    mlp3_scores.append(metricas_por_etiqueta(y_test, y_pred_mlp, 3))

    mlp_scores_cas1.append(metricas(y_test1, y_pred_mlp1))
    mlp_scores_cas2.append(metricas(y_test2, y_pred_mlp2))
    mlp_scores_cas3.append(metricas(y_test3, y_pred_mlp3))

# Convertir listas en matrices numpy para facilidad de cálculo
score_names = [nb_scores, nb0_scores, nb1_scores, nb2_scores, nb3_scores]
nb_scores, nb0_scores, nb1_scores, nb2_scores, nb3_scores = [np.array(score) for score in score_names]

score_names = [nb_scores_cas1, nb0_scores_cas1, nb1_scores_cas1, nb2_scores_cas1, nb3_scores_cas1]
nb_scores_cas1, nb0_scores_cas1, nb1_scores_cas1, nb2_scores_cas1, nb3_scores_cas1 = [np.array(score) for score in score_names]
score_names = [nb_scores_cas2, nb0_scores_cas2, nb1_scores_cas2, nb2_scores_cas2, nb3_scores_cas2]
nb_scores_cas2, nb0_scores_cas2, nb1_scores_cas2, nb2_scores_cas2, nb3_scores_cas2 = [np.array(score) for score in score_names]
score_names = [nb_scores_cas3, nb0_scores_cas3, nb1_scores_cas3, nb2_scores_cas3, nb3_scores_cas3]
nb_scores_cas3, nb0_scores_cas3, nb1_scores_cas3, nb2_scores_cas3, nb3_scores_cas3 = [np.array(score) for score in score_names]

score_names = [rf_scores, rf0_scores, rf1_scores, rf2_scores, rf3_scores]
rf_scores, rf0_scores, rf1_scores, rf2_scores, rf3_scores = [np.array(score) for score in score_names]
score_names = [rf_scores_cas1, rf0_scores_cas1, rf1_scores_cas1, rf2_scores_cas1, rf3_scores_cas1]
rf_scores_cas1, rf0_scores_cas1, rf1_scores_cas1, rf2_scores_cas1, rf3_scores_cas1 = [np.array(score) for score in score_names]
score_names = [rf_scores_cas2, rf0_scores_cas2, rf1_scores_cas2, rf2_scores_cas2, rf3_scores_cas2]
rf_scores_cas2, rf0_scores_cas2, rf1_scores_cas2, rf2_scores_cas2, rf3_scores_cas2 = [np.array(score) for score in score_names]
score_names = [rf_scores_cas3, rf0_scores_cas3, rf1_scores_cas3, rf2_scores_cas3, rf3_scores_cas3]
rf_scores_cas3, rf0_scores_cas3, rf1_scores_cas3, rf2_scores_cas3, rf3_scores_cas3 = [np.array(score) for score in score_names]

score_names = [svm_scores, svm0_scores, svm1_scores, svm2_scores, svm3_scores]
svm_scores, svm0_scores, svm1_scores, svm2_scores, svm3_scores = [np.array(score) for score in score_names]
score_names = [svm_scores_cas1, svm0_scores_cas1, svm1_scores_cas1, svm2_scores_cas1, svm3_scores_cas1]
svm_scores_cas1, svm0_scores_cas1, svm1_scores_cas1, svm2_scores_cas1, svm3_scores_cas1 = [np.array(score) for score in score_names]
score_names = [svm_scores_cas2, svm0_scores_cas2, svm1_scores_cas2, svm2_scores_cas2, svm3_scores_cas2]
svm_scores_cas2, svm0_scores_cas2, svm1_scores_cas2, svm2_scores_cas2, svm3_scores_cas2 = [np.array(score) for score in score_names]
score_names = [svm_scores_cas3, svm0_scores_cas3, svm1_scores_cas3, svm2_scores_cas3, svm3_scores_cas3]
svm_scores_cas3, svm0_scores_cas3, svm1_scores_cas3, svm2_scores_cas3, svm3_scores_cas3 = [np.array(score) for score in score_names]

score_names = [lr_scores, lr0_scores, lr1_scores, lr2_scores, lr3_scores]
lr_scores, lr0_scores, lr1_scores, lr2_scores, lr3_scores = [np.array(score) for score in score_names]
score_names = [lr_scores_cas1, lr0_scores_cas1, lr1_scores_cas1, lr2_scores_cas1, lr3_scores_cas1]
lr_scores_cas1, lr0_scores_cas1, lr1_scores_cas1, lr2_scores_cas1, lr3_scores_cas1 = [np.array(score) for score in score_names]
score_names = [lr_scores_cas2, lr0_scores_cas2, lr1_scores_cas2, lr2_scores_cas2, lr3_scores_cas2]
lr_scores_cas2, lr0_scores_cas2, lr1_scores_cas2, lr2_scores_cas2, lr3_scores_cas2 = [np.array(score) for score in score_names]
score_names = [lr_scores_cas3, lr0_scores_cas3, lr1_scores_cas3, lr2_scores_cas3, lr3_scores_cas3]
lr_scores_cas3, lr0_scores_cas3, lr1_scores_cas3, lr2_scores_cas3, lr3_scores_cas3 = [np.array(score) for score in score_names]

score_names = [mlp_scores, mlp0_scores, mlp1_scores, mlp2_scores, mlp3_scores]
mlp_scores, mlp0_scores, mlp1_scores, mlp2_scores, mlp3_scores = [np.array(score) for score in score_names]
score_names = [mlp_scores_cas1, mlp0_scores_cas1, mlp1_scores_cas1, mlp2_scores_cas1, mlp3_scores_cas1]
mlp_scores_cas1, mlp0_scores_cas1, mlp1_scores_cas1, mlp2_scores_cas1, mlp3_scores_cas1 = [np.array(score) for score in score_names]
score_names = [mlp_scores_cas2, mlp0_scores_cas2, mlp1_scores_cas2, mlp2_scores_cas2, mlp3_scores_cas2]
mlp_scores_cas2, mlp0_scores_cas2, mlp1_scores_cas2, mlp2_scores_cas2, mlp3_scores_cas2 = [np.array(score) for score in score_names]
score_names = [mlp_scores_cas3, mlp0_scores_cas3, mlp1_scores_cas3, mlp2_scores_cas3, mlp3_scores_cas3]
mlp_scores_cas3, mlp0_scores_cas3, mlp1_scores_cas3, mlp2_scores_cas3, mlp3_scores_cas3 = [np.array(score) for score in score_names]

# Calcular los promedios para Naive Bayes
promedio_f1_nb, promedio_acc_nb, promedio_matthews_nb, desviacion_f1_nb, desviacion_acc_nb, desviacion_matthews_nb = prom_desv(nb_scores)
promedio_f1_nb0, promedio_acc_nb0, promedio_matthews_nb0, desviacion_f1_nb0, desviacion_acc_nb0, desviacion_matthews_nb0 = prom_desv(nb0_scores)
promedio_f1_nb1, promedio_acc_nb1, promedio_matthews_nb1, desviacion_f1_nb1, desviacion_acc_nb1, desviacion_matthews_nb1 = prom_desv(nb1_scores)
promedio_f1_nb2, promedio_acc_nb2, promedio_matthews_nb2, desviacion_f1_nb2, desviacion_acc_nb2, desviacion_matthews_nb2 = prom_desv(nb2_scores)
promedio_f1_nb3, promedio_acc_nb3, promedio_matthews_nb3, desviacion_f1_nb3, desviacion_acc_nb3, desviacion_matthews_nb3 = prom_desv(nb3_scores)

# Calcular los promedios para Random Forest
promedio_f1_rf, promedio_acc_rf, promedio_matthews_rf, desviacion_f1_rf, desviacion_acc_rf, desviacion_matthews_rf = prom_desv(rf_scores)
promedio_f1_rf0, promedio_acc_rf0, promedio_matthews_rf0, desviacion_f1_rf0, desviacion_acc_rf0, desviacion_matthews_rf0 = prom_desv(rf0_scores)
promedio_f1_rf1, promedio_acc_rf1, promedio_matthews_rf1, desviacion_f1_rf1, desviacion_acc_rf1, desviacion_matthews_rf1 = prom_desv(rf1_scores)
promedio_f1_rf2, promedio_acc_rf2, promedio_matthews_rf2, desviacion_f1_rf2, desviacion_acc_rf2, desviacion_matthews_rf2 = prom_desv(rf2_scores)
promedio_f1_rf3, promedio_acc_rf3, promedio_matthews_rf3, desviacion_f1_rf3, desviacion_acc_rf3, desviacion_matthews_rf3 = prom_desv(rf3_scores)

# Calcular los promedios para SVM
promedio_f1_svm, promedio_acc_svm, promedio_matthews_svm, desviacion_f1_svm, desviacion_acc_svm, desviacion_matthews_svm = prom_desv(svm_scores)
promedio_f1_svm0, promedio_acc_svm0, promedio_matthews_svm0, desviacion_f1_svm0, desviacion_acc_svm0, desviacion_matthews_svm0 = prom_desv(svm0_scores)
promedio_f1_svm1, promedio_acc_svm1, promedio_matthews_svm1, desviacion_f1_svm1, desviacion_acc_svm1, desviacion_matthews_svm1 = prom_desv(svm1_scores)
promedio_f1_svm2, promedio_acc_svm2, promedio_matthews_svm2, desviacion_f1_svm2, desviacion_acc_svm2, desviacion_matthews_svm2 = prom_desv(svm2_scores)
promedio_f1_svm3, promedio_acc_svm3, promedio_matthews_svm3, desviacion_f1_svm3, desviacion_acc_svm3, desviacion_matthews_svm3 = prom_desv(svm3_scores)

# Calcular los promedios para Regresión Logística
promedio_f1_lr, promedio_acc_lr, promedio_matthews_lr, desviacion_f1_lr, desviacion_acc_lr, desviacion_matthews_lr = prom_desv(lr_scores)
promedio_f1_lr0, promedio_acc_lr0, promedio_matthews_lr0, desviacion_f1_lr0, desviacion_acc_lr0, desviacion_matthews_lr0 = prom_desv(lr0_scores)
promedio_f1_lr1, promedio_acc_lr1, promedio_matthews_lr1, desviacion_f1_lr1, desviacion_acc_lr1, desviacion_matthews_lr1 = prom_desv(lr1_scores)
promedio_f1_lr2, promedio_acc_lr2, promedio_matthews_lr2, desviacion_f1_lr2, desviacion_acc_lr2, desviacion_matthews_lr2 = prom_desv(lr2_scores)
promedio_f1_lr3, promedio_acc_lr3, promedio_matthews_lr3, desviacion_f1_lr3, desviacion_acc_lr3, desviacion_matthews_lr3 = prom_desv(lr3_scores)

# Calcular los promedios para ANN
promedio_f1_mlp, promedio_acc_mlp, promedio_matthews_mlp, desviacion_f1_mlp, desviacion_acc_mlp, desviacion_matthews_mlp = prom_desv(mlp_scores)
promedio_f1_mlp0, promedio_acc_mlp0, promedio_matthews_mlp0, desviacion_f1_mlp0, desviacion_acc_mlp0, desviacion_matthews_mlp0 = prom_desv(mlp0_scores)
promedio_f1_mlp1, promedio_acc_mlp1, promedio_matthews_mlp1, desviacion_f1_mlp1, desviacion_acc_mlp1, desviacion_matthews_mlp1 = prom_desv(mlp1_scores)
promedio_f1_mlp2, promedio_acc_mlp2, promedio_matthews_mlp2, desviacion_f1_mlp2, desviacion_acc_mlp2, desviacion_matthews_mlp2 = prom_desv(mlp2_scores)
promedio_f1_mlp3, promedio_acc_mlp3, promedio_matthews_mlp3, desviacion_f1_mlp3, desviacion_acc_mlp3, desviacion_matthews_mlp3 = prom_desv(mlp3_scores)

# -------------------------
# RESULTADOS FINALES
# -------------------------
from tabulate import tabulate

# Crear la tabla final con los promedios de cada modelo
tabla_resultados = [
    ["Naive Bayes", promedio_f1_nb, promedio_acc_nb, promedio_matthews_nb],
    ["Random Forest", promedio_f1_rf, promedio_acc_rf, promedio_matthews_rf],
    ["SVM", promedio_f1_svm, promedio_acc_svm, promedio_matthews_svm],
    ["Regresión Logística", promedio_f1_lr, promedio_acc_lr, promedio_matthews_lr],
    ["ANN", promedio_f1_mlp, promedio_acc_mlp, promedio_matthews_mlp]
]
tabla_resultados0 = [
    ["Naive Bayes", promedio_f1_nb0, promedio_acc_nb0, promedio_matthews_nb0],
    ["Random Forest", promedio_f1_rf0, promedio_acc_rf0, promedio_matthews_rf0],
    ["SVM", promedio_f1_svm0, promedio_acc_svm0, promedio_matthews_svm0],
    ["Regresión Logística", promedio_f1_lr0, promedio_acc_lr0, promedio_matthews_lr0],
    ["ANN", promedio_f1_mlp0, promedio_acc_mlp0, promedio_matthews_mlp0]
]
tabla_resultados1 = [
    ["Naive Bayes", promedio_f1_nb1, promedio_acc_nb1, promedio_matthews_nb1],
    ["Random Forest", promedio_f1_rf1, promedio_acc_rf1, promedio_matthews_rf1],
    ["SVM", promedio_f1_svm1, promedio_acc_svm1, promedio_matthews_svm1],
    ["Regresión Logística", promedio_f1_lr1, promedio_acc_lr1, promedio_matthews_lr1],
    ["ANN", promedio_f1_mlp1, promedio_acc_mlp1, promedio_matthews_mlp1]
]
tabla_resultados2 = [
    ["Naive Bayes", promedio_f1_nb2, promedio_acc_nb2, promedio_matthews_nb2],
    ["Random Forest", promedio_f1_rf2, promedio_acc_rf2, promedio_matthews_rf2],
    ["SVM", promedio_f1_svm2, promedio_acc_svm2, promedio_matthews_svm2],
    ["Regresión Logística", promedio_f1_lr2, promedio_acc_lr2, promedio_matthews_lr2],
    ["ANN", promedio_f1_mlp2, promedio_acc_mlp2, promedio_matthews_mlp2]
]
tabla_resultados3 = [
    ["Naive Bayes", promedio_f1_nb3, promedio_acc_nb3, promedio_matthews_nb3],
    ["Random Forest", promedio_f1_rf3, promedio_acc_rf3, promedio_matthews_rf3],
    ["SVM", promedio_f1_svm3, promedio_acc_svm3, promedio_matthews_svm3],
    ["Regresión Logística", promedio_f1_lr3, promedio_acc_lr3, promedio_matthews_lr3],
    ["ANN", promedio_f1_mlp3, promedio_acc_mlp3, promedio_matthews_mlp3]
]
# Imprimir la tabla con formato fancy_grid
print(tabulate(tabla_resultados, headers=["Modelo", "F1 Score", "Accuracy", "Coef. Matthews"], tablefmt="fancy_grid"))
print(tabulate(tabla_resultados0, headers=["Modelo", "F1 Score", "Accuracy", "Coef. Matthews"], tablefmt="fancy_grid"))
print(tabulate(tabla_resultados1, headers=["Modelo", "F1 Score", "Accuracy", "Coef. Matthews"], tablefmt="fancy_grid"))
print(tabulate(tabla_resultados2, headers=["Modelo", "F1 Score", "Accuracy", "Coef. Matthews"], tablefmt="fancy_grid"))
print(tabulate(tabla_resultados3, headers=["Modelo", "F1 Score", "Accuracy", "Coef. Matthews"], tablefmt="fancy_grid"))

# Crear subplots para cada métrica
fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(18, 6))
metric_names = ['F1-score', 'Accuracy', 'Matthews Correlation']
labels = ['Bayes', 'Random Forest', 'SVM', 'Regresión logística', 'ANN']
metric_statistics = []

# Iterar sobre las métricas
for i in range(3):
    data = [nb_scores[:, i], rf_scores[:, i], svm_scores[:, i], lr_scores[:, i], mlp_scores[:, i]]

    axes[i].boxplot(data, labels=labels)
    axes[i].set_title(metric_names[i])
    axes[i].set_ylabel('Score')

    for model, scores in zip(labels, data):
        stats = {
            'Modelo': model,
            'Métrica': metric_names[i],
            'Mínimo': np.min(scores),
            'Primer Cuartil (Q1)': np.percentile(scores, 25),
            'Mediana (Q2)': np.median(scores),
            'Tercer Cuartil (Q3)': np.percentile(scores, 75),
            'Máximo': np.max(scores),
            'Promedio': np.mean(scores)
        }
        metric_statistics.append(stats)

plt.tight_layout()
plt.show()  # Muestra el gráfico

# Mostrar las estadísticas de las métricas en un DataFrame
df_statistics = pd.DataFrame(metric_statistics)
print(df_statistics)

# Crear subplots para cada métrica
fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(18, 6))
metric_names = ['F1-score', 'Accuracy', 'Matthews Correlation']
labels = ['Bayes', 'Random Forest', 'SVM', 'Regresión logística', 'ANN']
metric_statistics = []  # Lista para almacenar las estadísticas

# Mostrar las estadísticas de las métricas en un DataFrame
df_statistics = pd.DataFrame(metric_statistics)
print(df_statistics)

# Iterar sobre las métricas
for i in range(3):
    # Datos para el diagrama de caja de bigotes
    data = [nb_scores_cas1[:, i], rf_scores_cas1[:, i], svm_scores_cas1[:, i], lr_scores_cas1[:, i], mlp_scores_cas1[:, i]]

    # Crear el diagrama de caja de bigotes
    axes[i].boxplot(data, labels=labels)
    axes[i].set_title(metric_names[i])
    axes[i].set_ylabel('Score')

    # Calcular y almacenar las estadísticas de cada modelo
    for model, scores in zip(labels, data):
        stats = {
            'Modelo': model,
            'Métrica': metric_names[i],
            'Mínimo': np.min(scores),
            'Primer Cuartil (Q1)': np.percentile(scores, 25),
            'Mediana (Q2)': np.median(scores),
            'Tercer Cuartil (Q3)': np.percentile(scores, 75),
            'Máximo': np.max(scores),
            'Promedio': np.mean(scores),
            'Incertdumbre:': np.median(scores)-np.percentile(scores, 25)
        }
        metric_statistics.append(stats)

# Ajustar el espaciado entre los subplots y mostrar el gráfico
plt.tight_layout()
plt.show()

# Mostrar las estadísticas de las métricas en un DataFrame
df_statistics = pd.DataFrame(metric_statistics)
print(df_statistics)

# Crear subplots para cada métrica
fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(18, 6))
metric_names = ['F1-score', 'Accuracy', 'Matthews Correlation']
labels = ['Bayes', 'Random Forest', 'SVM', 'Regresión logística', 'ANN']
metric_statistics = []  # Lista para almacenar las estadísticas

# Mostrar las estadísticas de las métricas en un DataFrame
df_statistics = pd.DataFrame(metric_statistics)
print(df_statistics)

# Iterar sobre las métricas
for i in range(3):
    # Datos para el diagrama de caja de bigotes
    data = [nb_scores_cas2[:, i], rf_scores_cas2[:, i], svm_scores_cas2[:, i], lr_scores_cas2[:, i], mlp_scores_cas2[:, i]]

    # Crear el diagrama de caja de bigotes
    axes[i].boxplot(data, labels=labels)
    axes[i].set_title(metric_names[i])
    axes[i].set_ylabel('Score')

    # Calcular y almacenar las estadísticas de cada modelo
    for model, scores in zip(labels, data):
        stats = {
            'Modelo': model,
            'Métrica': metric_names[i],
            'Mínimo': np.min(scores),
            'Primer Cuartil (Q1)': np.percentile(scores, 25),
            'Mediana (Q2)': np.median(scores),
            'Tercer Cuartil (Q3)': np.percentile(scores, 75),
            'Máximo': np.max(scores),
            'Promedio': np.mean(scores),
            'Incertdumbre:': np.median(scores)-np.percentile(scores, 25)
        }
        metric_statistics.append(stats)

# Ajustar el espaciado entre los subplots y mostrar el gráfico
plt.tight_layout()
plt.show()

# Mostrar las estadísticas de las métricas en un DataFrame
df_statistics = pd.DataFrame(metric_statistics)
print(df_statistics)

# Crear subplots para cada métrica
fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(18, 6))
metric_names = ['F1-score', 'Accuracy', 'Matthews Correlation']
labels = ['Bayes', 'Random Forest', 'SVM', 'Regresión logística', 'ANN']
metric_statistics = []  # Lista para almacenar las estadísticas

# Mostrar las estadísticas de las métricas en un DataFrame
df_statistics = pd.DataFrame(metric_statistics)
print(df_statistics)

# Iterar sobre las métricas
for i in range(3):
    # Datos para el diagrama de caja de bigotes
    data = [nb_scores_cas3[:, i], rf_scores_cas3[:, i], svm_scores_cas3[:, i], lr_scores_cas3[:, i], mlp_scores_cas3[:, i]]

    # Crear el diagrama de caja de bigotes
    axes[i].boxplot(data, labels=labels)
    axes[i].set_title(metric_names[i])
    axes[i].set_ylabel('Score')

    # Calcular y almacenar las estadísticas de cada modelo
    for model, scores in zip(labels, data):
        stats = {
            'Modelo': model,
            'Métrica': metric_names[i],
            'Mínimo': np.min(scores),
            'Primer Cuartil (Q1)': np.percentile(scores, 25),
            'Mediana (Q2)': np.median(scores),
            'Tercer Cuartil (Q3)': np.percentile(scores, 75),
            'Máximo': np.max(scores),
            'Promedio': np.mean(scores),
            'Incertdumbre:': np.median(scores)-np.percentile(scores, 25)
        }
        metric_statistics.append(stats)

# Ajustar el espaciado entre los subplots y mostrar el gráfico
plt.tight_layout()
plt.show()

# Mostrar las estadísticas de las métricas en un DataFrame
df_statistics = pd.DataFrame(metric_statistics)
print(df_statistics)

import numpy as np
import matplotlib.pyplot as plt

eventos = ['Reposo', 'Puño Izquierdo', 'Puño Derecho', 'Ambos Puños']
modelos = ['SVM', 'Regresión logistica', 'MLP', 'Naive Bayes', 'Random Forest']

# Accuracy promedio para cada modelo en cada evento
accuracy = np.array([
    [promedio_acc_svm0, promedio_acc_lr0, promedio_acc_mlp0, promedio_acc_nb0, promedio_acc_rf0],  # Precisión para la etiqueta 0
    [promedio_acc_svm1, promedio_acc_lr1, promedio_acc_mlp1, promedio_acc_nb1, promedio_acc_rf1],  # Precisión para la etiqueta 1
    [promedio_acc_svm2, promedio_acc_lr2, promedio_acc_mlp2, promedio_acc_nb2, promedio_acc_rf2],   # Precisión para la etiqueta 2
    [promedio_acc_svm3, promedio_acc_lr3, promedio_acc_mlp3, promedio_acc_nb3, promedio_acc_rf3]
])
f1_score = np.array([
    [promedio_f1_svm0, promedio_f1_lr0, promedio_f1_mlp0, promedio_f1_nb0, promedio_f1_rf0],  # Precisión para la etiqueta 0
    [promedio_f1_svm1, promedio_f1_lr1, promedio_f1_mlp1, promedio_f1_nb1, promedio_f1_rf1],  # Precisión para la etiqueta 1
    [promedio_f1_svm2, promedio_f1_lr2, promedio_f1_mlp2, promedio_f1_nb2, promedio_f1_rf2],   # Precisión para la etiqueta 2
    [promedio_f1_svm3, promedio_f1_lr3, promedio_f1_mlp3, promedio_f1_nb3, promedio_f1_rf3]
])
# Desviación estándar o error
error = np.array([
    [desviacion_acc_svm0, desviacion_acc_lr0, desviacion_acc_mlp0, desviacion_acc_nb0, desviacion_acc_rf0],
    [desviacion_acc_svm1, desviacion_acc_lr1, desviacion_acc_mlp1, desviacion_acc_nb1, desviacion_acc_rf1],
    [desviacion_acc_svm2, desviacion_acc_lr2, desviacion_acc_mlp2, desviacion_acc_nb2, desviacion_acc_rf2],
    [desviacion_acc_svm3, desviacion_acc_lr3, desviacion_acc_mlp3, desviacion_acc_nb3, desviacion_acc_rf3]
])
errorf1 = np.array([
    [desviacion_f1_svm0, desviacion_f1_lr0, desviacion_f1_mlp0, desviacion_f1_nb0, desviacion_f1_rf0],
    [desviacion_f1_svm1, desviacion_f1_lr1, desviacion_f1_mlp1, desviacion_f1_nb1, desviacion_f1_rf1],
    [desviacion_f1_svm2, desviacion_f1_lr2, desviacion_f1_mlp2, desviacion_f1_nb2, desviacion_f1_rf2],
    [desviacion_f1_svm3, desviacion_f1_lr3, desviacion_f1_mlp3, desviacion_f1_nb3, desviacion_f1_rf3]
])

# Crear el diagrama de barras agrupadas
x = np.arange(len(eventos))  # La posición de las barras
width = 0.15  # Ancho de las barras

# Crear la figura y los ejes
fig, ax = plt.subplots(figsize=(10, 6))

# Colores para los modelos
colores = ['#ff9a2e', '#ffd736', '#67dbf2', '#9467bd','#ff94f4']

# Dibujar las barras para cada modelo en cada evento
for i in range(len(modelos)):
    ax.bar(x + i * width, accuracy[:, i], width, label=modelos[i], color=colores[i], yerr=error[:, i], capsize=5)

# Etiquetas y título
ax.set_xlabel('Eventos')
ax.set_ylabel('Accuracy')
ax.set_title('Accuracy de Modelos por Evento')
ax.set_xticks(x + width * 2)
ax.set_xticklabels(eventos)
ax.legend(title='Modelos')

# Mostrar la gráfica
plt.tight_layout()
plt.show()

# Crear el diagrama de barras agrupadas
x = np.arange(len(eventos))  # La posición de las barras
width = 0.15  # Ancho de las barras

# Crear la figura y los ejes
fig, ax = plt.subplots(figsize=(10, 6))

# Colores para los modelos
colores = ['#ff9a2e', '#ffd736', '#67dbf2', '#9467bd','#ff94f4']

# Dibujar las barras para cada modelo en cada evento
for i in range(len(modelos)):
    ax.bar(x + i * width, f1_score[:, i], width, label=modelos[i], color=colores[i], yerr=errorf1[:, i], capsize=5)

# Etiquetas y título
ax.set_xlabel('Eventos')
ax.set_ylabel('F1 Score')
ax.set_title('F1 Score de Modelos por Evento')
ax.set_xticks(x + width * 2)
ax.set_xticklabels(eventos)
ax.legend(title='Modelos')

# Mostrar la gráfica
plt.tight_layout()
plt.show()

"""**# GRÁFICAS Y TABLA**"""

import pandas as pd
from IPython.display import display, HTML

# Crear la matriz de métricas como un diccionario
metricas = {
    'Modelo': ['Naive Bayes Gaussiano', 'Random Forest', 'SVM', 'Regresión Logística', 'ANN'],
    'F1 Score': [promedio_f1_nb, promedio_f1_rf, promedio_f1_svm, promedio_f1_lr, promedio_f1_mlp],
    'Accuracy': [promedio_acc_nb, promedio_acc_rf, promedio_acc_svm, promedio_acc_lr, promedio_acc_mlp],
    'Coef. Matthews': [promedio_matthews_nb, promedio_matthews_rf, promedio_matthews_svm, promedio_matthews_lr, promedio_matthews_mlp]
}

# Convertir el diccionario en un DataFrame
df_metricas = pd.DataFrame(metricas)

# Aplicar estilos y resaltar los valores más altos
styled_table = (
    df_metricas.style
    .highlight_max(subset=['F1 Score', 'Accuracy', 'Coef. Matthews'], color='lightgray')  # Resaltar los valores más altos
    .set_table_attributes('class="dataframe"')
    .set_table_styles([
        {'selector': 'th', 'props': [
            ('background-color', '#5db6d9'),
            ('color', 'white'),
            ('text-align', 'center'),
            ('border', '2px solid white')  # Agregar bordes al encabezado
        ]},
        {'selector': 'td', 'props': [
            ('text-align', 'center'),
            ('border', '1px solid white')  # Agregar bordes a las celdas
        ]},
        {'selector': 'tr:nth-child(even)', 'props': [('background-color', '#f2f2f2')]},  # Filas alternas
        {'selector': 'tr:hover', 'props': [('background-color', '#ddd')]}  # Efecto hover
    ])
)

# Mostrar la tabla
display(HTML(styled_table.to_html()))