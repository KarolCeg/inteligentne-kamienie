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

	float s = 0.16; //wspolczynnik skali, przyda sie do przemnazania wspolrzednych, musi byc typu float
	float dy = 5000;
	float dx = 1000;
	vector<Point2f>  punkty_na_pasach2D = { {dx * s, dy * s}, { (350 + dx) * s,(0 + dy) * s}, { (350 + dx) * s,(400 + dy) * s}, { (0 + dx) * s,(400 + dy) * s} };//UZUPELNIC
	Mat H1 = findHomography(punkty_na_obrazie, punkty_na_pasach2D);
	//sprawdzic czy macierz homografii sie zgadza z ta z instrukcji
	printf(H1, "homografia H1", 10, 4); //"czytelne" wypisanie macierzy
	cout << H1 << endl; //to samo z pelna dokladnoscia, choc mniej czytelne


	//sprawdzic czy homografia dziala - jaki wynik ponizszego przeksztalcenia
	//jest poprawny?
	cout << transformuj_wspolrzedne(438, 769, H1);
	Mat wyprostowany;
	warpPerspective(obraz_z_kamery, wyprostowany, H1, Size((dx + 300 + 350) * s, (dy + 400 + 250) * s)); //UZUPELNIC


	imshow("z kamery", obraz_z_kamery);
	setMouseCallback("z kamery", klikniecie_mysza_z_homografia, (void*)&H1);
	imshow("z lotu ptaka", wyprostowany);
	waitKey();


	//2. Zlokalizowac pasy wzgledem kamery. Do tego potrzebne s¹ parametry kamery - odczytac je z obrazka
	//kalibracyjnego. 
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

	//DODAC tu polecenia rysujace te 3 wersory
	line(obraz_z_kamery, wersory2D[0], wersory2D[1], Scalar(0, 0, 255), 2);
	line(obraz_z_kamery, wersory2D[0], wersory2D[2], Scalar(0, 255, 0), 2);
	line(obraz_z_kamery, wersory2D[0], wersory2D[3], Scalar(255, 0, 0), 2);

	Mat T_ok = T_ko.inv();
	printf(T_ok, "Polozenie kamery w ukladzie pasow:");

	//imshow("z kamery", obraz_z_kamery); //te dwa polecenie wyswietlaja obraz
	waitKey(); //i czekaja na nacisniecie klawisza

	//4. Uzupelnic
	vector<Point2f>  punkty_samochodowe2D = { {-1.3, 10.0}, { 2.0, 10.0}, { 2.0, 6.0}, { -1.3, 6.0 } };

	Mat H2 = findHomography(punkty_samochodowe2D, punkty_na_obrazie);
	Point2f wsp_obrazowe = transformuj_wspolrzedne(Point2f(-1.3, 10.0), H2);
	printf(H2, "H2", 10, 4);
	// i wyznaczyc H2,ktora przelicza wspolrzedne w metrach wzgledem pasow
	//na wspolrzedne pikselowe
	// 
	// 
	//5. Wykorzystujac H2 narysowac linie na obrazie z kamery co 10 metrow
	// PAMIETAC to wyswietleniu obrazu po narysowaniu czegos na nim!!!
	Mat kreski = obraz_z_kamery.clone();
	for (int i = 0; i < 6; i++) {
		Point2f p1 = transformuj_wspolrzedne(Point2f(-1.3, i * 10), H2);
		Point2f p2 = transformuj_wspolrzedne(Point2f(2.2, i * 10), H2);
		line(kreski, p1, p2, Scalar(0, 0, 255), 2);
	}
	//imshow("kreski", kreski); 




	//6. Wykorzystac zlozenie obu homografii do narysowania takich samych
	//linii na obrazie z lotu ptaka
	Mat homo_kreski = wyprostowany.clone();
	for (int i = 0; i < 6; i++) {
		Point2f p3 = transformuj_wspolrzedne(Point2f(-1.3, i * 10), H1 * H2);
		Point2f p4 = transformuj_wspolrzedne(Point2f(2.2, i * 10), H1 * H2);
		line(homo_kreski, p3, p4, Scalar(0, 0, 255), 2);
	}
	//imshow("homo_kreski", homo_kreski);

	//7. Narysowac bryly w miejscu wszystkich pasow na jezdni
	//uzyc zmiennych by latwo bylo zmienic wysokosc bryl

	Mat H3 = findHomography(punkty_samochodowe2D, punkty_na_pasach3D);
	Mat cube = obraz_z_kamery.clone();

	for (int i = 0; i < 6; i++) {


		vector<Point2f> podstawa = { { float(-1.3 + i * 0.93), 10 }, { float(-0.8 + i * 0.93), 10 }, { float(-0.8 + i * 0.93), 6 }, {float(-1.3 + i * 0.93), 6 } };
		for (auto& p : podstawa) p = transformuj_wspolrzedne(p, H3);
		vector<Point3f> podstawa_dolna3D;
		vector<Point3f> podstawa_gorna3D;
		for (auto p : podstawa) {
			podstawa_dolna3D.push_back(Point3f(p.x, p.y, 0));
			podstawa_gorna3D.push_back(Point3f(p.x, p.y, 50));
		}

		for (int i = 0; i < podstawa.size(); i++)
		{
			podstawa_dolna3D[i] = Point3f(podstawa[i].x, podstawa[i].y, 0);
			podstawa_gorna3D[i] = Point3f(podstawa[i].x, podstawa[i].y, 50);
		}

		vector<Point2f> podstawa_dolna2D;
		projectPoints(podstawa_dolna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen,
			podstawa_dolna2D);
		vector<Point2f> podstawa_gorna2D;
		projectPoints(podstawa_gorna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen,
			podstawa_gorna2D);


		line(cube, podstawa_dolna2D[0], podstawa_dolna2D[1], Scalar(255, 0, 0), 2);
		line(cube, podstawa_dolna2D[1], podstawa_dolna2D[2], Scalar(255, 0, 0), 2);
		line(cube, podstawa_dolna2D[2], podstawa_dolna2D[3], Scalar(255, 0, 0), 2);
		line(cube, podstawa_dolna2D[3], podstawa_dolna2D[0], Scalar(255, 0, 0), 2);

		line(cube, podstawa_gorna2D[0], podstawa_gorna2D[1], Scalar(255, 0, 0), 2);
		line(cube, podstawa_gorna2D[1], podstawa_gorna2D[2], Scalar(255, 0, 0), 2);
		line(cube, podstawa_gorna2D[2], podstawa_gorna2D[3], Scalar(255, 0, 0), 2);
		line(cube, podstawa_gorna2D[3], podstawa_gorna2D[0], Scalar(255, 0, 0), 2);

		line(cube, podstawa_dolna2D[1], podstawa_gorna2D[1], Scalar(255, 0, 0), 2);
		line(cube, podstawa_dolna2D[2], podstawa_gorna2D[2], Scalar(255, 0, 0), 2);
		line(cube, podstawa_dolna2D[3], podstawa_gorna2D[3], Scalar(255, 0, 0), 2);
		line(cube, podstawa_dolna2D[0], podstawa_gorna2D[0], Scalar(255, 0, 0), 2);

	}


	imshow("cube", cube);



	//8. Wykorzystac gleboka siec neuronowa YOLO do detekcji obiektow
	//i poeksperymentowac z jej parametrami
	Yolo yolo("yolov4-tiny.cfg", "yolov4-tiny.weights", "classes.txt", .2, .2);
	int k = 0;


	float width_samochod = 1.80;   // szerokość samochodu w metrach
	float length_samochod =4.50;  // długość samochodu w metrach
	float height_samochod = 1.50;  // wysokość samochodu w metrach

	float width_pieszy = 0.6;     // szerokość pieszego
	float length_pieszy = 0.8;    // długość pieszego (np. "głębokość" sylwetki)
	float height_pieszy = 1.7;    // wysokość pieszego
	Scalar color;

	while (k != 27)
	{
		kamera >> obraz_z_kamery;

		Mat obraz_skorygowany;
		undistort(obraz_z_kamery, obraz_skorygowany, macierzKamery, wspolczynnikiZnieksztalcen);

		warpPerspective(obraz_skorygowany, wyprostowany, H1, Size((350 + dx + 660) * s, (400 + dy + 300) * s));

		yolo.detect(obraz_skorygowany, Size(800, 800));
		yolo.draw_detections(obraz_skorygowany, {}, true);

		for (int klasa : {0, 1, 2, 3, 5, 7})
		{
			for (Rect wykryty_obiekt : yolo.valid_boxes[klasa])
			{

				float width, length, height;

				if (klasa == 0) {
					width = width_pieszy;
					length = length_pieszy;
					height = height_pieszy;
					color = Scalar(255, 0, 0);
				}
				else if (klasa == 2) {
					width = width_samochod;
					length = length_samochod;
					height = height_samochod;
					color = Scalar(0, 255, 0);
				}
				else {
					continue;  // pomiń nieobsługiwane klasy
				}

				Point2f prawy = wykryty_obiekt.br();
				Point2f lewy = wykryty_obiekt.tl() + Point(0, wykryty_obiekt.height);

				Point2f prawy_dolny;
				Point2f lewy_dolny;
				Point2f lewy_gorny;
				Point2f prawy_gorny;

				float odleglosc = 0.0;
				Point2f punkt;

				if (prawy.x > 450) {
					prawy_dolny = transformuj_wspolrzedne(prawy, H2.inv());
					lewy_dolny = prawy_dolny - Point2f(width, 0);
					lewy_gorny = lewy_dolny + Point2f(0, length);
					prawy_gorny = prawy_dolny + Point2f(0, length);
					odleglosc = prawy_dolny.y;
				}
				else {
					lewy_dolny = transformuj_wspolrzedne(lewy, H2.inv());
					prawy_dolny = lewy_dolny + Point2f(width, 0);
					lewy_gorny = lewy_dolny + Point2f(0, length);
					prawy_gorny = prawy_dolny + Point2f(0, length);
					odleglosc = lewy_dolny.y;
				}

				vector<Point2f> podstawa = { lewy_dolny, prawy_dolny, prawy_gorny, lewy_gorny };
				for (auto& p : podstawa) p = transformuj_wspolrzedne(p, H3);
				vector<Point3f> podstawa_dolna3D;
				vector<Point3f> podstawa_gorna3D;

				for (auto p : podstawa) {
					podstawa_dolna3D.push_back(Point3f(p.x, p.y, 0));
					podstawa_gorna3D.push_back(Point3f(p.x, p.y, height*100));
				}

				vector<Point2f> podstawa_dolna2D;
				vector<Point2f> podstawa_gorna2D;
				projectPoints(podstawa_dolna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen,podstawa_dolna2D);
				projectPoints(podstawa_gorna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen,podstawa_gorna2D);

				line(obraz_skorygowany, podstawa_dolna2D[0], podstawa_dolna2D[1], color,2);
				line(obraz_skorygowany, podstawa_dolna2D[1], podstawa_dolna2D[2], color, 2);
				line(obraz_skorygowany, podstawa_dolna2D[2], podstawa_dolna2D[3], color, 2);
				line(obraz_skorygowany, podstawa_dolna2D[3], podstawa_dolna2D[0], color, 2);


				line(obraz_skorygowany, podstawa_gorna2D[0], podstawa_gorna2D[1], color, 2);
				line(obraz_skorygowany, podstawa_gorna2D[1], podstawa_gorna2D[2], color, 2);
				line(obraz_skorygowany, podstawa_gorna2D[2], podstawa_gorna2D[3], color, 2);
				line(obraz_skorygowany, podstawa_gorna2D[3], podstawa_gorna2D[0], color, 2);

				line(obraz_skorygowany, podstawa_dolna2D[1], podstawa_gorna2D[1], color, 2);
				line(obraz_skorygowany, podstawa_dolna2D[2], podstawa_gorna2D[2], color, 2);
				line(obraz_skorygowany, podstawa_dolna2D[3], podstawa_gorna2D[3], color, 2);
				line(obraz_skorygowany, podstawa_dolna2D[0], podstawa_gorna2D[0], color, 2);

				line(wyprostowany, transformuj_wspolrzedne(lewy_dolny, H1 * H2), transformuj_wspolrzedne(prawy_dolny, H1 * H2), color, 2);
				line(wyprostowany, transformuj_wspolrzedne(lewy_dolny, H1 * H2), transformuj_wspolrzedne(lewy_gorny, H1 * H2), color, 2);
				line(wyprostowany, transformuj_wspolrzedne(lewy_gorny, H1 * H2), transformuj_wspolrzedne(prawy_gorny, H1 * H2), color, 2);
				line(wyprostowany, transformuj_wspolrzedne(prawy_dolny, H1 * H2), transformuj_wspolrzedne(prawy_gorny, H1 * H2), color, 2);

				punkt.x = (podstawa_dolna2D[1].x + podstawa_gorna2D[3].x) / 2;
				punkt.y = (podstawa_dolna2D[0].y + podstawa_gorna2D[2].y) / 2;
				putText(obraz_skorygowany, format("%.0f", odleglosc), punkt, 0, 1, CV_RGB(255, 50, 50), 2);
			}

			imshow("z kamery", obraz_skorygowany);
			imshow("z lotu ptaka", wyprostowany);
			k = waitKey(1);
			if (k == ' ') waitKey();
		}
	}
}
