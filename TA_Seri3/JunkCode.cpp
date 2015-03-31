#include "stdafx.h";
/*
for (int i=0 ; i<cam ; i++){
				for (int j = 0 ; j < cam ; j++){
					if (i!=j && KF[i].trackStat && KF[j].trackStat){
						//masukin semua kandidat yang beririsan--------------------
						for (int sz = 0; sz < ObjAll[i].size() ;  sz++){
							if (cekGrad(Ls[i][j],ObjAll[i].at(sz).bottom)>0){
								greaterLs[i][j].push_back(ObjAll[i].at(sz));
							}
						}
						//----------------------------------------------------------
					}
				}
			}
*/

/*PASANGIN X sama Y-----------------------------------------------------
			if (!learnFOV)
			for (int x=0 ; x<cam ; x++){
				for (int y=0 ; y<cam ; y++){
					if ((x<y) && ((greaterLs[x][y].size())==(greaterLs[y][x].size())) && greaterLs[x][y].size()==1 && (cekGrad(Ls[x][y],Obj[x].bottom)>0 || (cekGrad(Ls[y][x],Obj[y].bottom)>0))){
					//if ((x<y) && (KF[x].trackStat && KF[y].trackStat) && ((greaterLs[x][y].size()==1 && ObjAll[y].size()>0 && cekGrad(Ls[x][y],Obj[x].bottom)>0) || (greaterLs[y][x].size()==1 && ObjAll[x].size()>0 && cekGrad(Ls[y][x],Obj[y].bottom)>0))){
						if (KF[x].lock && KF[y].lock==false && (cekGrad(Ls[x][y],Obj[x].bottom)>0)){
							SelectionSort(greaterLs[x][y],Ls[x][y]);
							int posisi = binarySearch(greaterLs[x][y],Obj[x]);
							
							pilihPointCam(Ls[x][y],greaterLs[y][x],frame_size,posisi);
							Obj[y] = greaterLs[y][x].at(posisi);
							
							KF[y].framelewat = KF[x].framelewat;
							resetKalman(KF[y].KF,KF[y].measurement,Obj[y].pusat);
							std::stringstream out;
							std::stringstream out1;
							out << x;
							out1 << y;
							KF[y].Label = "Obj"+out.str()+""+out1.str();
							KF[x].Label = "Obj"+out.str()+""+out1.str();
						}
						if(KF[y].lock && KF[x].lock==false && (cekGrad(Ls[y][x],Obj[y].bottom)>0)){
							SelectionSort(greaterLs[y][x],Ls[y][x]);
							int posisi = binarySearch(greaterLs[y][x],Obj[y]);
							
							pilihPointCam(Ls[y][x],greaterLs[x][y],frame_size,posisi);
							Obj[x] = greaterLs[x][y].at(posisi);
							
							KF[x].framelewat = KF[y].framelewat;
							resetKalman(KF[x].KF,KF[x].measurement,Obj[x].pusat);
							std::stringstream out;
							std::stringstream out1;
							out << x;
							out1 << y;
							KF[y].Label = "Obj"+out1.str()+""+out.str();
							KF[x].Label = "Obj"+out1.str()+""+out.str();
						}
					}
				}
			}
*/

/*
			for (int i=0 ; i<cam ; i++){
				if (KF[i].lock==true){
					KF[i].locklama = true;
				}else{
					KF[i].locklama = false;
				}
			}
			
			for (int i = 0 ; i < cam ; i++){
				for (int j = 0 ; j < cam ; j++){
					if (i!=j && KF[i].trackStat && KF[j].trackStat){
						if (cekGrad(Ls[i][j],Obj[i].bottom)>0){
							for (int sz = 0; sz < ObjAll[i].size() ;  sz++){
								if (cekGrad(Ls[i][j],ObjAll[i].at(sz).bottom)>0){
									greaterLs[i][j].push_back(ObjAll[i].at(sz));
								}
							}
							
							SelectionSort(greaterLs[i][j],Ls[i][j]);
							
							if ((greaterLs[i][j].size()==1) && (KF[j].lock==false)){
								int posI = 0;
								KObject KFtemp = KF[j];
								u_koresponden.stimulus = true;
								pilihPointCam(Ls[i][j],ObjAll[j],frame_size,posI);
								Obj[j] = ObjAll[j].at(posI);
								resetKalman(KF[j].KF,KF[j].measurement,Obj[j].pusat);
								lockj.push_back(j);
								std::stringstream out;
								std::stringstream out1;
								out << i;
								out1 << j;
								if (KF[i].framelewat>0){
									KF[j].Label = KF[i].Label;
									int rad = abs(distance_to_point(KF[j].bottom,KF[j].pusat));
									if (rad < abs(distance_to_point(KF[j].pusat,KFtemp.pusat))){
										KF[i].Label = "null";
									}
									KF[j].framelewat = 0;
								}else{
									KF[j].Label = "Obj C"+out.str()+""+out1.str();
								}
								
								//if (!KF[i].locklama){
								//	KF[i].Label="null";
								//}
								
							}
						}
					}
					greaterLs[i][j].clear();
				}
				KF[i].LabelPoint = Point(KF[i].pusat.x, (KF[i].pusat.y+(KF[i].height/2)));
				cv::putText(duplicate[i],KF[i].Label, KF[i].LabelPoint,1,1,CV_RGB(255, 0, 0),1,8,false);
				
			}
			
			for (int a=0; a<lockj.size() ; a++){
				KF[lockj.at(a)].lock = true;
			}
			lockj.clear();
			*/