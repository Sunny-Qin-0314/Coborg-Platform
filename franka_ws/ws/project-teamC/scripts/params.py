

def validate():
    global pegboard #allows us to call and update pegboard like a class

    pegboard = [0,0,0,0] #init a list with 4 index

    for i in range(len((pegboard))):
        pegboard[i] = int(input('What tool is in pegboard position {}? [1, 2, 3, 4, or 0 for none]'.format(i)))

    # TO DO:
    #
    # This function is validation for what tools are currently on the pegboard.
    # We need to know what tools are in what locations on the pegboard. 
    # Return a list/array that shows tool locations in the tool holder
    # 0 = Empty, 1 = Screwdriver, 2 = Hammer, 3 = Wrench
    #
    # Example: Let's say there is a Screwdriver, Nothing, Nothing, Hammer on the pegboard.
    #
    # Ask the user: What is in the first location? '1'
    # Second location? '0'
    # Third location? '0'
    # Fourth location? '2'
    #
    # store a list with [1, 0, 0, 2] for access later
    # Now, when we call "Robot pick screwdriver" it knows there's a screwdriver in index 0.
    # Also if we say "Robot place wrench" it knows that locations 1 & 2 are empty
 
if __name__ == "__main__": #unit testing code goes here
    pass