#pragma once

#include "opencv2\opencv.hpp"




using namespace cv;
using namespace std;


extern vector<Point2f> punkty_na_obrazie;
extern vector<Point2f> H_punkty_klikniete;
extern vector<Point2f> H_punkty_przeliczone;
extern  Mat obraz_z_kamery;


int ulica()
{
	//Wgrac pierwsza klatke z filmu ulice4.mp4
	VideoCapture kamera("ulice4.mp4");
	kamera >> obraz_z_kamery;
	if (!kamera.isOpened())
	{
		cout << "\n\nNie mozna zaladowac pliku z filmem, pewnie go nie ma w lokalnym katalogu? \n\nNacisnij ENTER by zakonczyc program!";
		cin.get();
		return 0;
	} //jak film sie nie zaladuje, program od razu zakonczy dzialanie


	//1. Zastosowac homografie do uzyskania widoku z lotu ptaka - ma byæ widac pas wlasny i dwa sasiednie
	// na 50 metrow przed pasami - wymiary pasow wyraziæ w CENTYMETRACH

	punkty_na_obrazie.push_back(Point(438, 769));
	punkty_na_obrazie.push_back(Point(977, 773));
	punkty_na_obrazie.push_back(Point(1150, 870));
	punkty_na_obrazie.push_back(Point(335, 867));

	//koleczka raczej opcjonalnie w celu weryfikacji czy wpisane wspolrzedne sa poprawne
	//circle(obraz_z_kamery, punkty_na_obrazie[0], 5, CV_RGB(255, 0, 0), 1);
	//circle(obraz_z_kamery, punkty_na_obrazie[1], 5, CV_RGB(255, 0, 0), 1);
	//circle(obraz_z_kamery, punkty_na_obrazie[2], 5, CV_RGB(255, 0, 0), 1);
	//circle(obraz_z_kamery, punkty_na_obrazie[3], 5, CV_RGB(255, 0, 0), 1);

	//wspolrzedne punktow na pasach w cm UZUPELNIC POPRAWNIE

	float s = 0.15; //wspolczynnik skali, przyda sie do przemnazania wspolrzednych, musi byc typu float
	float dx = 500;
	float dy = 5000;

	vector<Point2f>  punkty_na_pasach2D = {
		{ (0 + dx) * s,   (0 + dy) * s },
		{ (350 + dx) * s, (0 + dy) * s },
		{ (350 + dx) * s, (400 + dy) * s },
		{ (0 + dx) * s, (400 + dy) * s }
	};//UZUPELNIC
	Mat H1 = findHomography(punkty_na_obrazie, punkty_na_pasach2D);
	//sprawdzic czy macierz homografii sie zgadza z ta z instrukcji
	printf(H1, "homografia H1", 10, 4); //"czytelne" wypisanie macierzy
	cout << H1 << endl; //to samo z pelna dokladnoscia, choc mniej czytelne

	//sprawdzic czy homografia dziala - jaki wynik ponizszego przeksztalcenia
	//jest poprawny?
	cout << transformuj_wspolrzedne(438, 769, H1);
	Mat wyprostowany;
	warpPerspective(obraz_z_kamery, wyprostowany, H1, Size((350 + dx + 250) * s, (400 + dy + 300) * s)); //UZUPELNIC


	imshow("z kamery", obraz_z_kamery);
	//setMouseCallback("z kamery", klikniecie_mysza_z_homografia, (void*)&H1);

	imshow("z lotu ptaka", wyprostowany);


	//2. Zlokalizowac pasy wzgledem kamery. Do tego potrzebne s¹ parametry kamery - odczytac je z obrazka
	//kalibracyjnego. 
	float y1 = 400;
	vector<Point3f>  punkty_na_pasach3D = { {0,0,0}, {350,0,0}, {350,-400,0}, {0,-400,0} }; //UZUPELNIC

	Mat trans, rot; //wektory translacji i rotacji obiektu
	solvePnP(punkty_na_pasach3D, punkty_na_obrazie, macierzKamery, wspolczynnikiZnieksztalcen, rot, trans);
	//tworzenie macierzy przeksztalcenia jednorodnego - polozenie obiektu w ukladzie kamery
	Mat T_ko = Mat::eye(4, 4, CV_64F); //macierz jednostkowa
	Rodrigues(rot, T_ko(Rect(0, 0, 3, 3))); //przeksztalcenie wektora rotacji na klasyczna macierz rotacji i skopiowanie jej do macierzy transformacji uogolnionej
	Mat(trans).copyTo(T_ko(Rect(3, 0, 1, 3))); //skopiowanie wektora translacji do macierzy T
	printf(T_ko, "Macierz przeksztalcenie jednorodnego (polozenie obiektu w ukladzie kamery):");

	//3. Narysowac wersory osi
	vector<Point3f>  wersory3D = { {0,0,0}, {100,0,0}, {0,100,0}, {0,0,100} };
	vector<Point2f>  wersory2D;
	projectPoints(wersory3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, wersory2D);
	line(obraz_z_kamery, wersory2D[0], wersory2D[1], CV_RGB(255, 0, 0), 2);
	line(obraz_z_kamery, wersory2D[0], wersory2D[2], CV_RGB(0, 255, 0), 2);
	line(obraz_z_kamery, wersory2D[0], wersory2D[3], CV_RGB(0, 0, 255), 2);

	imshow("z kamery", obraz_z_kamery); //te dwa polecenie wyswietlaja obraz

	Mat T_ok = T_ko.inv();
	printf(T_ok, "Polozenie kamery w ukladzie pasow:");
	//i czekaja na nacisniecie klawisza
   //4. Uzupelnic
	vector<Point2f>  punkty_samochodowe2D = { {-1.3, 10}, { 2.2, 10}, { 2.2, 6}, { -1.3, 6 } };
	Mat H2 = findHomography(punkty_samochodowe2D, punkty_na_obrazie);

	// i wyznaczyc H2,ktora przelicza wspolrzedne w metrach wzgledem pasow
	//na wspolrzedne pikselowe

	//5. Wykorzystujac H2 narysowac linie na obrazie z kamery co 10 metrow
	vector<vector<Point2f>> wsp_obrazowe_linii;
	for (int odleglosc = 10; odleglosc <= 50; odleglosc += 10)
	{
		vector<Point2f> pojedyncza_linia;
		Point2f lewy = transformuj_wspolrzedne(Point2f(-1.3, odleglosc), H2);
		Point2f prawy = transformuj_wspolrzedne(Point2f(2.2, odleglosc), H2);
		pojedyncza_linia.push_back(lewy);
		pojedyncza_linia.push_back(prawy);
		wsp_obrazowe_linii.push_back(pojedyncza_linia);
		line(obraz_z_kamery, lewy, prawy, CV_RGB(255, 0, 0), 2);
	}
	for (int i = 0; i < wsp_obrazowe_linii.size(); i++)
	{
		line(obraz_z_kamery, wsp_obrazowe_linii[i][0], wsp_obrazowe_linii[i][1], CV_RGB(255, 0, 0), 2);
	}
	imshow("z kamery", obraz_z_kamery);

	// PAMIETAC to wyswietleniu obrazu po narysowaniu czegos na nim!!!
	//6. Wykorzystac zlozenie obu homografii do narysowania takich samych
	//linii na obrazie z lotu ptaka
	Mat H12 = H1 * H2;

	for (int odleglosc = 10; odleglosc <= 50; odleglosc += 10)
	{
		Point2f lewy_wyprostowany = transformuj_wspolrzedne(Point2f(-1.3, odleglosc), H12);
		Point2f prawy_wyprostowany = transformuj_wspolrzedne(Point2f(2.2, odleglosc), H12);

		line(wyprostowany, lewy_wyprostowany, prawy_wyprostowany, CV_RGB(255, 0, 0), 2);
	}
	imshow("z lotu ptaka", wyprostowany);

	//7. Narysowac bryly w miejscu wszystkich pasow na jezdni
	//uzyc zmiennych by latwo bylo zmienic wysokosc bryl
	Mat H3 = findHomography(punkty_samochodowe2D, punkty_na_pasach3D);
	for (int pas = 0; pas < 6; pas++)
	{
		float przesuniecie_x = pas * 1;
		vector<Point2f> podstawa = {
			{ float(-1.3 + przesuniecie_x), 10 },
			{ float(-0.8 + przesuniecie_x), 10 },
			{ float(-0.8 + przesuniecie_x), 6 },
			{ float(-1.3 + przesuniecie_x), 6 }
		};
		for (auto& p : podstawa) p = transformuj_wspolrzedne(p, H3);
		vector<Point3f> podstawa_dolna3D;
		vector<Point3f> podstawa_gorna3D;

		for (auto p : podstawa)
		{
			podstawa_dolna3D.push_back(Point3f(p.x, p.y, 0));
			podstawa_gorna3D.push_back(Point3f(p.x, p.y, 50));
		}

		vector<Point2f> podstawa_dolna2D;
		projectPoints(podstawa_dolna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, podstawa_dolna2D);

		vector<Point2f> podstawa_gorna2D;
		projectPoints(podstawa_gorna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, podstawa_gorna2D);

		for (int i = 0; i < 4; i++)
		{
			int nastepny = (i + 1) % 4;
			line(obraz_z_kamery, podstawa_dolna2D[i], podstawa_dolna2D[nastepny], CV_RGB(0, 0, 255), 2);
			line(obraz_z_kamery, podstawa_gorna2D[i], podstawa_gorna2D[nastepny], CV_RGB(0, 0, 255), 2);
			line(obraz_z_kamery, podstawa_dolna2D[i], podstawa_gorna2D[i], CV_RGB(0, 0, 255), 2);
		}
	}
	imshow("z kamery", obraz_z_kamery);

	//8. Wykorzystac gleboka siec neuronowa YOLO do detekcji obiektow
	//i poeksperymentowac z jej parametrami
	Yolo yolo("yolov4-tiny.cfg", "yolov4-tiny.weights", "classes.txt", .2, .2);
	int k = 0;
	while (k != 27)
	{
		kamera >> obraz_z_kamery;
		warpPerspective(obraz_z_kamery, wyprostowany, H1, Size((350 + dx + 250) * s, (400 + dy + 300) * s));

		yolo.detect(obraz_z_kamery, Size(800, 800));
		yolo.draw_detections(obraz_z_kamery, {}, true);

		for (int klasa : {0, 1, 2, 3, 5, 7})
		{
			for (Rect wykryty_obiekt : yolo.valid_boxes[klasa])
			{
				// 1. Środek dolnej krawędzi ramki (punkt na asfalcie)
				Point2f dolny_srodek_piksele(wykryty_obiekt.tl().x + wykryty_obiekt.width / 2.0f, wykryty_obiekt.br().y);
				Point2f srodek_metry = transformuj_wspolrzedne(dolny_srodek_piksele, H2.inv());

				// Wypisywanie odległości
				float odleglosc = srodek_metry.y;
				putText(obraz_z_kamery, format("%.1f m", odleglosc), wykryty_obiekt.tl(), 0, 0.6, CV_RGB(0, 255, 0), 2);

				// Znacznik na widoku z lotu ptaka
				Mat H12 = H1 * H2;
				Point2f p_lot = transformuj_wspolrzedne(srodek_metry, H12);
				line(wyprostowany, Point2f(p_lot.x - 25, p_lot.y), Point2f(p_lot.x + 25, p_lot.y), CV_RGB(0, 255, 0), 2);

				// --- USTAWIENIA GABARYTÓW W ZALEŻNOŚCI OD KLASY (CZŁOWIEK VS POJAZD) ---
				float szerokosc_obiektu, dlugosc_obiektu, wysokosc_obiektu;
				Scalar kolor_linii;

				if (klasa == 0) {
					// Wymiary dla CZŁOWIEKA
					szerokosc_obiektu = 0.6f;  // Szerokość barków: 60 cm
					dlugosc_obiektu = 0.6f;    // Głębokość ciała: 60 cm
					wysokosc_obiektu = 180.0f; // Wysokość: 1.8 metra (180 cm dla osi Z)
					kolor_linii = CV_RGB(0, 255, 255); // Kolor jasnoniebieski (cyjan)
				}
				else {
					// Wymiary dla POJAZDÓW
					szerokosc_obiektu = 1.8f;  // Szerokość auta: 1.8 m
					dlugosc_obiektu = 4.5f;    // Długość auta: 4.5 m
					wysokosc_obiektu = 150.0f; // Wysokość auta: 1.5 metra (150 cm dla osi Z)
					kolor_linii = CV_RGB(255, 165, 0); // Kolor pomarańczowy
				}

				// --- RYSOWANIE PROSTOPADŁOŚCIANU 3D ---
				float x_lewy = srodek_metry.x - (szerokosc_obiektu / 2.0f);
				float x_prawy = srodek_metry.x + (szerokosc_obiektu / 2.0f);
				float y_tyl = srodek_metry.y;
				float y_przod = y_tyl + dlugosc_obiektu;

				vector<Point2f> podstawa_obiektu = {
					{ x_lewy, y_tyl },
					{ x_lewy, y_przod },
					{ x_prawy, y_przod },
					{ x_prawy, y_tyl }
				};

				// Przeliczanie na 3D z użyciem H3
				for (auto& p : podstawa_obiektu) {
					p = transformuj_wspolrzedne(p, H3);
				}

				// Dodanie osi Z (wysokość dopasowana do obiektu)
				vector<Point3f> podstawa_dolna3D, podstawa_gorna3D;
				for (auto p : podstawa_obiektu) {
					podstawa_dolna3D.push_back(Point3f(p.x, p.y, 0));
					podstawa_gorna3D.push_back(Point3f(p.x, p.y, wysokosc_obiektu));
				}

				// Rzutowanie na ekran
				vector<Point2f> podstawa_dolna2D, podstawa_gorna2D;
				projectPoints(podstawa_dolna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, podstawa_dolna2D);
				projectPoints(podstawa_gorna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, podstawa_gorna2D);

				// Rysowanie 12 krawędzi odpowiednim kolorem
				for (int i = 0; i < 4; i++) {
					int nastepny = (i + 1) % 4;
					line(obraz_z_kamery, podstawa_dolna2D[i], podstawa_dolna2D[nastepny], kolor_linii, 2);
					line(obraz_z_kamery, podstawa_gorna2D[i], podstawa_gorna2D[nastepny], kolor_linii, 2);
					line(obraz_z_kamery, podstawa_dolna2D[i], podstawa_gorna2D[i], kolor_linii, 2);
				}
			}
		}

		// Rysowanie siatki na ulicy (odległości co 10 metrów)
		for (int odleglosc = 10; odleglosc <= 50; odleglosc += 10)
		{
			vector<Point2f> pojedyncza_linia;
			Point2f lewy = transformuj_wspolrzedne(Point2f(-1.3, odleglosc), H2);
			Point2f prawy = transformuj_wspolrzedne(Point2f(2.2, odleglosc), H2);
			pojedyncza_linia.push_back(lewy);
			pojedyncza_linia.push_back(prawy);
			wsp_obrazowe_linii.push_back(pojedyncza_linia);
		}

		for (int odleglosc = 10; odleglosc <= 50; odleglosc += 10)
		{
			Point2f lewy_wyprostowany = transformuj_wspolrzedne(Point2f(-1.3, odleglosc), H12);
			Point2f prawy_wyprostowany = transformuj_wspolrzedne(Point2f(2.2, odleglosc), H12);

			line(wyprostowany, lewy_wyprostowany, prawy_wyprostowany, CV_RGB(255, 0, 0), 2);
		}

		imshow("z kamery", obraz_z_kamery);
		imshow("z lotu ptaka", wyprostowany);
		k = waitKey(1);
		if (k == ' ') waitKey();
	}
}
