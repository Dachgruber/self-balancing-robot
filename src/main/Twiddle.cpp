/* Self balancing Robot via Stepper Motor with microstepping and Digital   Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
*/
#include   "Twiddle.h"
/**********************************************************************/
Twiddle::Twiddle(   int anzParams, float p0 , float p1, float p2, float p3, float p4 ,
                  float   p5, float p6, float p7,
                  float dp0, float dp1, float dp2, float   dp3 , float dp4,
                  float dp5, float dp6 , float dp7)
/**********************************************************************/
{
   params[0]  = p0;
  params[1]  = p1;
  params[2]  = p2;
  params[3]  =   p3;
  params[4]  = p4;
  params[5]  = p5;
  params[6]  = p6;
  params[7]   = p7;
  dparams[0] = dp0;
  dparams[1] = dp1;
  dparams[2] = dp2;
   dparams[3] = dp3;
  dparams[4] = dp4;
  dparams[5] = dp5;
  dparams[6]   = dp6;
  dparams[7] = dp7;
  index = 0;
  besterr = 9999.99;
  nextStep   = 1;
  AnzahlElements = anzParams;
}
/**********************************************************************/
Twiddle::Twiddle(   int anzParams, float iparams[], float idparams[]  )   // Constructor
/**********************************************************************/
{   index = 0;
  besterr = 9999.99;
  nextStep = 1;
  AnzahlElements = anzParams;
}
/**********************************************************************/
float   Twiddle::next(float error,  float &p0, float &p1, float &p2, float &p3,
                    float   &p4, float &p5, float &p6, float &p7)
/**********************************************************************/
{
   static int skip = 0;

  sum_average += abs(error);
  cnt_average ++;

   if (skip > 5 ||  // for collection some data
      skip == 0 ) { //first Time
     skip = 1;
    if ( cnt_average > 0 ) {
      average = sum_average / cnt_average;
       sum_average = 0;
      cnt_average = 0;
    }
  }
  else {
     skip ++;
    return average;
  }

  if (( dparams[0] +  dparams[1]   +  dparams[2] +  dparams[3] +  ( dparams[4] +  dparams[5] +  dparams[6] +  dparams[7]))   < 0.03 ) {
    //   Serial.println(" erledigt ");
    p0 = params[0];
     p1 = params[1];
    p2 = params[2];
    p3 = params[3];
    p4 = params[4];
     p5 = params[5];
    p6 = params[6];
    p7 = params[7];

    //   try again
    dparams[0] *= 4.0;
    dparams[1] *= 4.0;
    dparams[2]   *= 4.0;
    dparams[3] *= 4.0;
    dparams[4] *= 4.0;
    dparams[5] *=   4.0;
    dparams[6] *= 4.0;
    dparams[7] *= 4.0;
    besterr = 9999.99;

     return average;  // it is done
  }


  if (  nextStep == 3 ) {
     nextStep = 1;
    if (index == AnzahlElements - 1) {
      index = 0;
     } else {
      index ++;
    }
    params[index] +=  dparams[index];
   }

  logging(); // last step

  calcCost(average);

  p0 = params[0];
   p1 = params[1];
  p2 = params[2];
  p3 = params[3];
  p4 = params[4];
   p5 = params[5];
  p6 = params[6];
  p7 = params[7];
  return average;
}
//----------------------------------------------------------------
void   Twiddle::calcCost(float average)
//----------------------------------------------------------------
//   Dieser Algorithmus sucht nun den Parameterraum intelligent ab und
// variiert   die Schrittweite der Suche, je nachdem ob man in der Nhe
// eines Maxima bzw.   Minima ist.
{

  switch (nextStep) {
    case 1:
      if (average   < besterr) {
        // aktuelle Kosten < besterr # There was some improvement
         besterr = average;
        //mit grerem Schritt probieren
        dparams[index]   *= 1.1;
        nextStep = 3;
      } else // # There was no improvement
       {
        // # Go into the other direction
        params[index] =  params[index]   - (2 * dparams[index]);
        nextStep = 2;
      }
      break;

     case 2:
      if (average < besterr) {
        // aktuelle Kosten < besterr   # There was some improvement
        besterr = average;
        //mit grerem   Schritt probieren
        dparams[index] *= 1.05;
        nextStep = 1;
       } else {
        // As there was no improvement, the step size in either
         // direction, the step size might simply be too big.
        params[index]   += dparams[index];
        dparams[index] *=  0.95;//an sonsten kleineren Schritt
         nextStep = 3;
      }
      break;
  }
}
/*------------------------------------------------------------
   # Choose an initialization parameter vector
  p = [0, 0, 0]
  # Define potential   changes
  dp = [1, 1, 1]
  # Calculate the error
  best_err = A(p)
   threshold = 0.001
  while sum(dp) > threshold:
    for i in range(len(p)):
         p[i] += dp[i]
        err = A(p)
        if err < best_err:  # There   was some improvement
            best_err = err
            dp[i] *= 1.1
         else:  # There was no improvement
            p[i] -= 2*dp[i]  # Go into   the other direction
            err = A(p)
            if err < best_err:   # There was an improvement
                best_err = err
                dp[i]   *= 1.05
            else  # There was no improvement
                p[i]   += dp[i]
                # As there was no improvement, the step size in either
                 # direction, the step size might simply be too big.
                dp[i]   *= 0.95

  https://www.gomatlab.de/twiddle-algorithmus-zum-optimieren-von-parametern-t24517.html
    % Maximierung der Kostenfunktion!
  while sum(dparams) > 0.01
    for i=1:length(params)   % alle Parameter durch gehen
        params(i)=params(i)+dparams(i);
        %Kostenfunktion   ausrechnen
        cfzz(it) = calcCost(params(1),params(2));
        if cfzz(it)   > besterr
            besterr = cfzz(it);
            dparams(i)= dparams(i)*1.05;
         else
            % in andere Richtung suchen
            params(i)=params(i)-   2*dparams(i);
            cfzz(it) = calcCost(params(1),params(2));
            if   cfzz(it) > besterr %wenn aktuelle Kosten hher (=gut)
                besterr   = cfzz(it);
                dparams(i) = dparams(i)*1.05; %mit grerem Schritt   probieren
            else
                params(i)=params(i)+dparams(i);
                 dparams(i)=dparams(i)*0.95; % an sonsten kleineren Schritt
            end
         end
    it = it+1;
   end
*/

//----------------------------------------------------------------
void   Twiddle::logging()
//----------------------------------------------------------------
{

   Serial.print(" Step= ");
  Serial.print(nextStep );
  Serial.print("   Ind= ");
  Serial.print(index );
  Serial.print(" av= ");
  Serial.print(average   , 4 );
  Serial.print(" besterr ");
  Serial.print(besterr , 4 );
  Serial.print("   P0 ");
  Serial.print(params[0]  , 4 );
  Serial.print(" P1 ");
  Serial.print(params[1]   , 4);
  Serial.print(" P2 ");
  Serial.print(params[2]  , 4 );
  Serial.print("   P3 ");
  Serial.print(params[3]  , 4 );
  Serial.print(" P4 ");
  Serial.print(params[4]   , 2 );
  Serial.print(" P5 ");
  Serial.print(params[5]  , 2);
  Serial.print("   P6 ");
  Serial.print(params[6]  , 2 );
  Serial.print(" P7 ");
  Serial.print(params[7]   , 4 );
  Serial.println(" ");
}
