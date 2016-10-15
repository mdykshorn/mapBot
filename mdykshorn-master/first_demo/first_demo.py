#!/usr/bin/python

# Demo of some programming concepts in Python
# Initial code written by: Jason Ziglar <jpz@vt.edu>
# Extended and modified by: Morgan Dykshorn

#imports the math library
import math

# Define a function which computes the harmonic series
def harmonic_series(num_terms):
  '''Function which computes some number of terms from the harmonic series.
  num_terms: the number of terms to add together.
  returns: The value computed'''
  
  

  #checks to make sure the number of terms is above 0
  if num_terms <1:
    return 0
  else:
  #Create variable to store result, set to some initial value
    value = 0

    #range produces a sequence of numbers, [0, num_terms), and executes the loop
    # with ii set to each value

    for ii in range(num_terms):
        #Set value to the value plus the next term
        # Note: Computers start counting at 0, so we have to add 1 to be safe
        value = value + (1.0 / (ii + 1))

    # Return the value to whomever called this function
    return value

def harmonic_series_while(num_terms):
  '''Function which computes some number of terms from the harmonic series, using a while loop.
  num_terms: the number of terms to add together.
  returns: The sum of the first n terms of the series'''

  

  #checks to make sure the number of terms is above 0
  if num_terms <1:
    return 0
  else:
    #Create variable to store summation
    value = 0
    #A counter to keep track of how many terms have been computed
    counter = 0

    # This will run until "counter <= num_terms" returns a false statement
    while counter < num_terms:
     value = value + (1.0 / (counter + 1))
        # Very important - while runs until it sees false, so we have to make sure
        # the test will eventually fail
     counter = counter + 1

  # Return value
  return value


def pi_approx(num_terms):
    '''Function which approximates pi to n number of iterations.
    num_terms: the number of terms added together
    returns: Pi'''

    #checks to make sure the number of terms is above 0
    if num_terms <1:
      return 0
    else:
        #Create variable to store summation
        value = 0
        #A counter to keep track of how many terms have been computed
        counter = 0

        # This will run until "counter <= num_terms" returns a false statement
        while counter < num_terms:
             value = value + (1.0 / (counter + 1)**2 )
             #do calculations to get pi
     
            # Very important - while runs until it sees false, so we have to make sure
            # the test will eventually fail
             counter = counter + 1

    value = math.sqrt(value*6)
    return value


if __name__ == "__main__":
  #Starting here, the program begins execution, since the previous statements were describing functions, but not actually calling them
  # Print a welcome
  print "Welcome to a simple harmonic series approximation program."

  endprogram = 0

  while endprogram !=4:

      #Ask the user to select a function
      request = raw_input("Select which function to use: 1) For loop 2) While loop 3) Pi Approximation 4) End Function: ")
      #Convert that input to an integer
      request = int(request)

      endprogram = request

      #checks to make sure a function was chosen
      if request <1 or request >4:
       print "Invalid Function Chosen"
      #checks if the number is 4 which ends the program
      elif request ==4:
          quit()
      else:
          #Same as before, but in a single line.
          iterations = float(raw_input("How many terms should I use? "))
          #convert iterations to an int
          iterations = int(iterations)

          endprogram1 = iterations


          # Test input
          if request == 1:
              # Get value form function
              result = harmonic_series(iterations)
              # Print using a technique known as string interpolation. %s means "take the next value after the string and insert as a string"
              # So it will look at the list of values after the % and grab the next (only) one
              # For more details, look here: https://docs.python.org/2/library/stdtypes.html#string-formatting
              print "For loop produces: %s" % result
          elif request == 2:
              # Same as previous statement, but notice you don't have to store a value in a variable before using it, if it makes sense.
              print "While loop produces %s" % harmonic_series_while(iterations)
          else:
              #runs function to calculate pi
              print "Pi Approximation produces %s" %pi_approx(iterations)