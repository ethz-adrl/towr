Begin  TO NLP problem

* Lines with the asterix are ignored
* This files must be located where the executable is started from
* and the name of the file has to be passed to the snopt problem in code as:
* setSpecsFile  ( "snopt.spc" );
*
* A complete list of options can be found in the snopt user guide:
* https://web.stanford.edu/group/SOL/guides/sndoc7.pdf

   Major Print level           000001
   Minor print level                1
   Solution                       yes
   
* Some options that terminate snopt early   
*  Major iterations limit           6
*  Iterations limit                 100000
   
* These options strongly influce SNOPT based on the FAQ.snopt
* file located in the downloaded source   
*   Crash option       3
*   Hessian updates    5

* something about gradient approximation
* Nonderivative linesearch

* Convergence Tolerances 
  Major feasibility tolerance 1.0e-2  *target nonlinear constraint violation
  Major optimality tolerance 1.0e-2   *target complementarity gap 
  Minor feasibility tolerance 1.0e-2  *for satisfying the QP bounds

End  TO NLP problem
