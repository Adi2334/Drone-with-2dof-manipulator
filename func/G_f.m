function G = G_f(X,Y,Z,PHI,THETA,PSI,TH1,TH2)

G = ([0.0;0.0;8.80938;cos(TH1+TH2).*cos(THETA).*sin(PHI).*4.905e-2+cos(TH1).*cos(THETA).*sin(PHI).*1.4715e-1;sin(TH1+TH2).*cos(THETA).*4.905e-2+cos(THETA).*sin(TH1).*1.4715e-1+cos(TH1+TH2).*cos(PHI).*sin(THETA).*4.905e-2+cos(PHI).*cos(TH1).*sin(THETA).*1.4715e-1;0.0;cos(TH1+TH2).*sin(THETA).*4.905e-2+cos(TH1).*sin(THETA).*1.4715e-1+sin(TH1+TH2).*cos(PHI).*cos(THETA).*4.905e-2+cos(PHI).*cos(THETA).*sin(TH1).*1.4715e-1;cos(TH1+TH2).*sin(THETA).*4.905e-2+sin(TH1+TH2).*cos(PHI).*cos(THETA).*4.905e-2]);

end