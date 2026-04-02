#pragma once

#include "opencv2\opencv.hpp"
#include <filesystem>




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
	
	// pusc film od n-tej klatki
	int start_frame = 4000; 
	kamera.set(CAP_PROP_POS_FRAMES, start_frame);

	Size imageSize = obraz_z_kamery.size();

	// zniwelowanie znieksztalcen soczewkowych===============================================================================
	Mat map1, map2;
	cv::initUndistortRectifyMap(macierzKamery, wspolczynnikiZnieksztalcen, Mat(), macierzKamery, imageSize, CV_16SC2, map1, map2);
	Mat obraz_oczyszczony;
	cv::remap(obraz_z_kamery, obraz_oczyszczony, map1, map2, INTER_LINEAR);

	string folder_path = "C:/Users/PA/Desktop/wizja_2024stud/wizja_2024stud";
	string file_name = "obraz_oczyszczony.jpg";  
	string full_path = folder_path + file_name; 

	// Zapis obrazu do wyznaczenia nowych puntkow z usunieciem znieksztalcen soczewkowych
	// cv::imwrite(full_path, obraz_oczyszczony);

	//imshow("Obraz po korekcji", obraz_oczyszczony);
	//waitKey(0);
	
	// ===========================================================================================================
	// 
	//1. Zastosowac homografie do uzyskania widoku z lotu ptaka - ma by  widac pas wlasny i dwa sasiednie
	// na 50 metrow przed pasami - wymiary pasow wyrazi  w CENTYMETRACH

	punkty_na_obrazie.push_back(Point(450, 763));
	punkty_na_obrazie.push_back(Point(980, 773));
	punkty_na_obrazie.push_back(Point(1148, 866));
	punkty_na_obrazie.push_back(Point(352, 859));

	// to sa te z instrukcji bez usuwania znieksztalcen soczewkowych !!!!!!!!!!
	/*
	punkty_na_obrazie.push_back(Point(438, 769));
	punkty_na_obrazie.push_back(Point(977, 773));
	punkty_na_obrazie.push_back(Point(1150, 870));
	punkty_na_obrazie.push_back(Point(335, 867));*/

	//koleczka raczej opcjonalnie w celu weryfikacji czy wpisane wspolrzedne sa poprawne
	//circle(obraz_z_kamery, punkty_na_obrazie[0], 5, CV_RGB(255, 0, 0), 1);
	//circle(obraz_z_kamery, punkty_na_obrazie[1], 5, CV_RGB(255, 0, 0), 1);
	//circle(obraz_z_kamery, punkty_na_obrazie[2], 5, CV_RGB(255, 0, 0), 1);
	//circle(obraz_z_kamery, punkty_na_obrazie[3], 5, CV_RGB(255, 0, 0), 1);

	//wspolrzedne punktow na pasach w cm UZUPELNIC POPRAWNIE

	float s = 0.15; //wspolczynnik skali, przyda sie do przemnazania wspolrzednych, musi byc typu float
	int dx = 500;
	int dy = 5000;
	vector<Point2f>  punkty_na_pasach2D = { {dx * s, dy * s}, { (dx + 350) * s,dy * s}, { (dx + 350) * s,(dy + 400) * s}, { dx * s,(dy + 400) * s } };
	Mat H1 = findHomography(punkty_na_obrazie, punkty_na_pasach2D);
	//sprawdzic czy macierz homografii sie zgadza z ta z instrukcji
	//printf(H1, "homografia H1",10,4); //"czytelne" wypisanie macierzy
	//cout << H1 << endl; //to samo z pelna dokladnoscia, choc mniej czytelne

	//sprawdzic czy homografia dziala - jaki wynik ponizszego przeksztalcenia
	//jest poprawny?
	//cout << transformuj_wspolrzedne(438, 769, H1);
	Mat wyprostowany;
	warpPerspective(obraz_z_kamery, wyprostowany, H1, Size((350 + dx + 220) * s, (400 + dy + 300) * s)); //UZUPELNIC


	//imshow("z kamery", obraz_z_kamery);
	//imshow("z lotu ptaka", wyprostowany);
	//waitKey();


	//2. Zlokalizowac pasy wzgledem kamery. Do tego potrzebne s  parametry kamery - odczytac je z obrazka
	//kalibracyjnego. 
	vector<Point3f>  punkty_na_pasach3D = { {0,0,0}, {350,0,0}, {350,-400,0}, {0,-400,0} }; //UZUPELNIC

	Mat trans, rot; //wektory translacji i rotacji obiektu
	solvePnP(punkty_na_pasach3D, punkty_na_obrazie, macierzKamery, wspolczynnikiZnieksztalcen, rot, trans);
	//tworzenie macierzy przeksztalcenia jednorodnego - polozenie obiektu w ukladzie kamery
	Mat T_ko = Mat::eye(4, 4, CV_64F); //macierz jednostkowa
	Rodrigues(rot, T_ko(Rect(0, 0, 3, 3))); //przeksztalcenie wektora rotacji na klasyczna macierz rotacji i skopiowanie jej do macierzy transformacji uogolnionej
	Mat(trans).copyTo(T_ko(Rect(3, 0, 1, 3))); //skopiowanie wektora translacji do macierzy T
	//printf(T_ko, "Macierz przeksztalcenie jednorodnego (polozenie obiektu w ukladzie kamery):");


	//3. Narysowac wersory osi
	vector<Point3f>  wersory3D = { {0,0,0}, {100,0,0}, {0,100,0}, {0,0,100} };
	vector<Point2f>  wersory2D;
	projectPoints(wersory3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, wersory2D);
	//DODAC tu polecenia rysujace te 3 wersory
	//line(obraz_z_kamery, wersory2D[0], wersory2D[1], Scalar(0, 0, 255), 2);
	//line(obraz_z_kamery, wersory2D[0], wersory2D[2], Scalar(0, 255, 0), 2);
	//line(obraz_z_kamery, wersory2D[0], wersory2D[3], Scalar(255, 0, 0), 2);

	Mat T_ok = T_ko.inv();
	printf(T_ok, "Polozenie kamery w ukladzie pasow:", 10, 4);


	//imshow("z kamery", obraz_z_kamery); //te dwa polecenie wyswietlaja obraz
	//waitKey(); //i czekaja na nacisniecie klawisza



	//4. Uzupelnic	========================================================================================== kreski
	vector<Point2f>  punkty_samochodowe2D = { {-1.3, 10.0}, { 2.2, 10.0}, { 2.2, 6.0}, { -1.3, 6.0 } };
	// i wyznaczyc H2,ktora przelicza wspolrzedne w metrach wzgledem pasow
	//na wspolrzedne pikselowe
	Mat H2 = findHomography(punkty_samochodowe2D, punkty_na_obrazie);
	printf(H2, "H2", 10, 4);
	Point2f wsp_obrazowe = transformuj_wspolrzedne(Point2f(-1.3, 10), H2);

	Mat kreseczki = obraz_z_kamery.clone();
	Mat kreseczki_wyprostowane = wyprostowany.clone();


	for (int i = 0; i < 10; ++i) {
		line(obraz_z_kamery, transformuj_wspolrzedne(Point2f(-1.3, i * 10), H2), transformuj_wspolrzedne(Point2f(2, i * 10), H2), Scalar(0, 0, 255), 1);
		line(wyprostowany, transformuj_wspolrzedne(Point2f(-1.3, i * 10), H1 * H2), transformuj_wspolrzedne(Point2f(2, i * 10), H1 * H2), Scalar(0, 0, 255), 1);
	}
	//imshow("kreseczki", kreseczki); //te dwa polecenie wyswietlaja obraz
	//imshow("kreseczki 2", kreseczki_wyprostowane); //te dwa polecenie wyswietlaja obraz
	//waitKey(); //i czekaja na nacisniecie klawisza



	//5. Wykorzystujac H2 narysowac linie na obrazie z kamery co 10 metrow
	// PAMIETAC to wyswietleniu obrazu po narysowaniu czegos na nim!!!

	//6. Wykorzystac zlozenie obu homografii do narysowania takich samych
	//linii na obrazie z lotu ptaka

	//7. Narysowac bryly w miejscu wszystkich pasow na jezdni
	//uzyc zmiennych by latwo bylo zmienic wysokosc bryl


	Mat H3 = findHomography(punkty_samochodowe2D, punkty_na_pasach3D);
	Mat szesciany = obraz_z_kamery.clone();

	for (int i = 0; i < 6; ++i) {
		vector<Point2f> podstawa = { { float(-1.3 + i * 1), 10 }, { float(-0.8 + i * 1), 10 }, { float(-0.8 + i * 1), 6 }, { float(-1.3 + i * 1), 6 } };

		for (auto& p : podstawa) p = transformuj_wspolrzedne(p, H3);

		vector<Point3f> podstawa_dolna3D;
		vector<Point3f> podstawa_gorna3D;
		for (auto p : podstawa)
		{
			podstawa_dolna3D.push_back(Point3f(p.x, p.y, 0));
			podstawa_gorna3D.push_back(Point3f(p.x, p.y, 50));
		}

		vector<Point2f> podstawa_dolna2D;
		vector<Point2f> podstawa_gorna2D;
		projectPoints(podstawa_dolna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, podstawa_dolna2D);
		projectPoints(podstawa_gorna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, podstawa_gorna2D);

		for (int i = 0; i < 4; i++)
		{
			line(szesciany, podstawa_dolna2D[i], podstawa_dolna2D[(i + 1) % 4], Scalar(255, 0, 0), 2);
		}
		for (int i = 0; i < 4; i++)
		{
			line(szesciany, podstawa_gorna2D[i], podstawa_gorna2D[(i + 1) % 4], Scalar(255, 0, 0), 2);
		}
		for (int i = 0; i < 4; i++)
		{
			line(szesciany, podstawa_dolna2D[i], podstawa_gorna2D[i], Scalar(255, 0, 0), 2);
		}
	}

	//imshow("szesciany", szesciany);
	//waitKey();



	//8. Wykorzystac gleboka siec neuronowa YOLO do detekcji obiektow
	//i poeksperymentowac z jej parametrami
	Yolo yolo("yolov4-tiny.cfg", "yolov4-tiny.weights", "classes.txt", .2, .2);
	int k = 0;

	// dane dla samochodow
	float width = 1.80;
	float height = 1.50;
	float length = 4.50;

	//dane dla ludzi
	float person_width = 0.5;
	float person_height = 2.0;
	float person_length = 0.5;

	while (k != 27)
	{
		kamera >> obraz_z_kamery;
		warpPerspective(obraz_z_kamery, wyprostowany, H1, Size((350 + dx + 220) * s, (400 + dy + 300) * s));
		
		// dodanie skorygowanego obrazu
		cv::remap(obraz_z_kamery, obraz_oczyszczony, map1, map2, INTER_LINEAR);
		obraz_z_kamery = obraz_oczyszczony;

		yolo.detect(obraz_z_kamery, Size(800, 800));
		yolo.draw_detections(obraz_z_kamery, {}, true);

		for (int i = 0; i < 6; ++i) {
			line(obraz_z_kamery, transformuj_wspolrzedne(Point2f(-1.3, i * 10), H2), transformuj_wspolrzedne(Point2f(2, i * 10), H2), Scalar(0, 0, 255), 2);
			line(wyprostowany, transformuj_wspolrzedne(Point2f(-1.3, i * 10), H1 * H2), transformuj_wspolrzedne(Point2f(2, i * 10), H1 * H2), Scalar(0, 0, 255), 2);
		}


		for (int klasa : {0, 1, 2, 3, 5, 7}) {
			for (Rect wykryty_obiekt : yolo.valid_boxes[klasa]) {

				if (klasa == 2) {
					Point2f cos = wykryty_obiekt.br();
					Point2f cos2 = wykryty_obiekt.tl() + Point(0, wykryty_obiekt.height);

					Point2f prawy_dolny;
					Point2f lewy_dolny;
					Point2f lewy_gorny;
					Point2f prawy_gorny;

					float odleglosc_od_obiektu = 0.0;
					Point2f punkt_wypisania;

					if (cos.x > 400) { // wykrycie co jest z ktorej strony
						cout << "Prawy dolny: " << cos << "\n"; 
						prawy_dolny = transformuj_wspolrzedne(cos, H2.inv());
						cout << "Po homografii: " << prawy_dolny << "\n";
						lewy_dolny = prawy_dolny - Point2f(width, 0);
						lewy_gorny = lewy_dolny + Point2f(0, length);
						prawy_gorny = prawy_dolny + Point2f(0, length);
						odleglosc_od_obiektu = prawy_dolny.y;
					} else {
						cout << "Lewy dolny: " << cos2 << "\n"; 
						lewy_dolny = transformuj_wspolrzedne(cos2, H2.inv());
						cout << "Po homografii: " << lewy_dolny << "\n";
						prawy_dolny = lewy_dolny + Point2f(width, 0);
						lewy_gorny = lewy_dolny + Point2f(0, length);
						prawy_gorny = prawy_dolny + Point2f(0, length);
						odleglosc_od_obiektu = lewy_dolny.y;
					}


					vector<Point2f> podstawa = { lewy_dolny, prawy_dolny, prawy_gorny, lewy_gorny };


					for (auto& p : podstawa) p = transformuj_wspolrzedne(p, H3);


					vector<Point3f> podstawa_dolna3D, podstawa_gorna3D;
					for (auto p : podstawa) {
						podstawa_dolna3D.push_back(Point3f(p.x, p.y, 0));
						podstawa_gorna3D.push_back(Point3f(p.x, p.y, height * 100));
					}


					vector<Point2f> podstawa_dolna2D, podstawa_gorna2D;
					projectPoints(podstawa_dolna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, podstawa_dolna2D);
					projectPoints(podstawa_gorna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, podstawa_gorna2D);


					for (int i = 0; i < 4; i++) {
						line(obraz_z_kamery, podstawa_dolna2D[i], podstawa_dolna2D[(i + 1) % 4], Scalar(255, 0, 0), 2);
						line(obraz_z_kamery, podstawa_gorna2D[i], podstawa_gorna2D[(i + 1) % 4], Scalar(255, 0, 0), 2);
						line(obraz_z_kamery, podstawa_dolna2D[i], podstawa_gorna2D[i], Scalar(255, 0, 0), 2);
					}

					// punkt do wypisywania tekstu
					punkt_wypisania.x = (podstawa_dolna2D[1].x + podstawa_gorna2D[3].x) / 2;
					punkt_wypisania.y = (podstawa_dolna2D[1].y + podstawa_gorna2D[3].y) / 2;

					// wypisanie odleglosci
					putText(obraz_z_kamery, format("%.0f", odleglosc_od_obiektu), punkt_wypisania, 0, 1, CV_RGB(155, 50, 50), 2);

					// rysowanie prostakatow wokol samochodow z lotu ptaka
					line(wyprostowany, transformuj_wspolrzedne(lewy_dolny, H1* H2), transformuj_wspolrzedne(prawy_dolny, H1* H2), Scalar(0, 255, 0), 2);
					line(wyprostowany, transformuj_wspolrzedne(lewy_dolny, H1* H2), transformuj_wspolrzedne(lewy_gorny, H1* H2), Scalar(0, 255, 0), 2);
					line(wyprostowany, transformuj_wspolrzedne(lewy_gorny, H1* H2), transformuj_wspolrzedne(prawy_gorny, H1* H2), Scalar(0, 255, 0), 2);
					line(wyprostowany, transformuj_wspolrzedne(prawy_dolny, H1* H2), transformuj_wspolrzedne(prawy_gorny, H1* H2), Scalar(0, 255, 0), 2);
				}

				if (klasa == 0) {
					Point2f cos = wykryty_obiekt.br();
					Point2f cos2 = wykryty_obiekt.tl() + Point(0, wykryty_obiekt.height);

					Point2f prawy_dolny;
					Point2f lewy_dolny;
					Point2f lewy_gorny;
					Point2f prawy_gorny;

					float odleglosc_od_obiektu = 0.0;
					Point2f punkt_wypisania;

					if (cos.x > 500) { // wykrycie co jest z ktorej strony
						cout << "Prawy dolny: " << cos << "\n"; 
						prawy_dolny = transformuj_wspolrzedne(cos, H2.inv());
						cout << "Po homografii: " << prawy_dolny << "\n";
						lewy_dolny = prawy_dolny - Point2f(person_width, 0);
						lewy_gorny = lewy_dolny + Point2f(0, person_length);
						prawy_gorny = prawy_dolny + Point2f(0, person_length);
						odleglosc_od_obiektu = prawy_dolny.y;
					} else {
						cout << "Lewy dolny: " << cos2 << "\n"; 
						lewy_dolny = transformuj_wspolrzedne(cos2, H2.inv());
						cout << "Po homografii: " << lewy_dolny << "\n";
						prawy_dolny = lewy_dolny + Point2f(person_width, 0);
						lewy_gorny = lewy_dolny + Point2f(0, person_length);
						prawy_gorny = prawy_dolny + Point2f(0, person_length);
						odleglosc_od_obiektu = lewy_dolny.y;
					}


					vector<Point2f> podstawa = { lewy_dolny, prawy_dolny, prawy_gorny, lewy_gorny };


					for (auto& p : podstawa) p = transformuj_wspolrzedne(p, H3);


					vector<Point3f> podstawa_dolna3D, podstawa_gorna3D;
					for (auto p : podstawa) {
						podstawa_dolna3D.push_back(Point3f(p.x, p.y, 0));
						podstawa_gorna3D.push_back(Point3f(p.x, p.y, person_height * 100));
					}

					vector<Point2f> podstawa_dolna2D, podstawa_gorna2D;
					projectPoints(podstawa_dolna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, podstawa_dolna2D);
					projectPoints(podstawa_gorna3D, rot, trans, macierzKamery, wspolczynnikiZnieksztalcen, podstawa_gorna2D);


					for (int i = 0; i < 4; i++) {
						line(obraz_z_kamery, podstawa_dolna2D[i], podstawa_dolna2D[(i + 1) % 4], Scalar(255, 3, 184), 2);
						line(obraz_z_kamery, podstawa_gorna2D[i], podstawa_gorna2D[(i + 1) % 4], Scalar(255, 3, 184), 2);
						line(obraz_z_kamery, podstawa_dolna2D[i], podstawa_gorna2D[i], Scalar(255, 3, 184), 2);
					}

					// punkt do wypisywania tekstu
					punkt_wypisania.x = (podstawa_dolna2D[1].x + podstawa_gorna2D[3].x) / 2;
					punkt_wypisania.y = (podstawa_dolna2D[1].y + podstawa_gorna2D[3].y) / 2;

					// wypisanie odleglosci
					putText(obraz_z_kamery, format("%.0f", odleglosc_od_obiektu), punkt_wypisania, 0, 1, CV_RGB(155, 50, 50), 2);

					// rysowanie prostakatow wokol czlowieka z lotu ptaka
					line(wyprostowany, transformuj_wspolrzedne(lewy_dolny, H1* H2), transformuj_wspolrzedne(prawy_dolny, H1* H2), Scalar(255, 3, 184), 2);
					line(wyprostowany, transformuj_wspolrzedne(lewy_dolny, H1* H2), transformuj_wspolrzedne(lewy_gorny, H1* H2), Scalar(255, 3, 184), 2);
					line(wyprostowany, transformuj_wspolrzedne(lewy_gorny, H1* H2), transformuj_wspolrzedne(prawy_gorny, H1* H2), Scalar(255, 3, 184), 2);
					line(wyprostowany, transformuj_wspolrzedne(prawy_dolny, H1* H2), transformuj_wspolrzedne(prawy_gorny, H1* H2), Scalar(255, 3, 184), 2);
				}
				// problem z czlowiekiem ze jest przesuniety chodzi o znieksztalecenia soczewkowe
			}
		}

		imshow("z kamery", obraz_z_kamery);
		imshow("z lotu ptaka", wyprostowany);

		// cv::remap(obraz_z_kamery, obraz_oczyszczony, map1, map2, INTER_LINEAR);
		// imshow("Obraz po korekcji", obraz_oczyszczony);
		// waitKey(0);


		k = waitKey(1);
		if (k == ' ') waitKey();
	}
}
