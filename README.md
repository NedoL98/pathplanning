# PathPlanning.ru :: автоматическое планирование траектории
Недолужко Андрей Алексеевич, группа 161-1

## Актуальность задачи
Задача планирования оптимальной траектории на плоскости возникает во множестве ситуаций - от необходимости автомобилисту найти кратчайший путь до дома до реализации искусственного интеллекта в компьютерной игре. Данная проблема возникает при разработке приложений, связанных с оцифровкой карт местности (Яндекс.Карты, Google Maps и т.д.), или при создании тех же компьютерных игр. 

## Обзор существующих решений
На данный момент самым распространённым решением данной задачи является применение так или иначе модифицированного алгоритма А*. Данный алгоритм является наиболее эффективным алгоритмом как по времени работы, так и по затрачиваемой памяти. 

## Технологические решения
При разработке проекта используются следующие технологии:

- C++ 

Широко распространённый объектно-ориентированный язык программирования, с помощью которого и будет реализована программа для решения задачи. Является компилируемым, следовательно, имеет преимущество в производительности перед другими языками.
- Qt Creator

Удобная IDE, свободно распространяемая для научных и учебных проектов.
- XML

Инструмент для задания входных/выходных данных. Является кроссплатформенным, файл с таким расширением можно открыть практически на любом современном устройстве. 

## План работы по реализации функциональности проекта 
- Изучить материалы по поисковым алгоритмам.
- Написать программу, в которой реализован какой-то алгоритм поиска, считывание карты и вывод пути в лог-файл. 
- Реализовать алгоритм A*, исследовать эффективность различных эвристический функций.

# Формат входных данных
Входные данные представляют из себя XML-файл. Он разделён на три основные части:

## Map

- width, height - соответственно ширина и высота карты.
- cellsize - размер одной ячейки в условных единицах. Является необязательным параметром.
- startx, starty, finishx, finishy - соответственно координаты старта и финиша.
- grid - описание карты. Проходимые области обозначены нулями, непроходимые - единицами.

## Algorithm

- searchtype - алгоритм, используемый для поиска пути. Например, A* (astar), Theta* (theta), JPS (jp_search), Dijkstra (djkstra).
- metrictype - заданная на плоскости метрика. Например, Евклидова метрика (Euclidean), Манхэтеннская метрика (Manhattan), расстояние Чебышева (Chebyshev) и диагональное расстояние (Diagonal).
- breakingties - параметр, отвечающий за алгоритм выбора новой вершины в случае, когда F- значения нескольких вершин совпадают.
- hweight - весовой параметр эвристики.
- allowdiagonal - возможность строить пути, проходящие по диагонали (а не только по квадратной сетке).
- cutcorners - возможность "срезать углы", как показано на рисунке:

![Alt text](/images/corner.jpg)

- allowsqueeze - возможность проходить между двумя препятствиями, как показано на рисунке:

![Alt text](/images/squeze.jpg)

## Options

- loglevel - "уровень логирования" отвечает за подробность лог-файла.
- logpath, logfilename - необязательные параметры, отвечающие соответственно за то, где создаётся лог-файл, и как он называется.
