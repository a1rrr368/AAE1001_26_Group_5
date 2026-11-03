import math
# C = C(f) . dF . T(best) .  + C(t) . T(best) + C(c)
# where C(f) is cost of fuel per kg, dF is the trip fuel, C(t) is the time related cost per minute of flight, C(c) is the fixed cost indepedent of time
## constant term: T(best) = 58.7875 minutes
## Based on scenario 1
## C(f) = 0.85 $/kg

Tb = 58.7875
Cf = 0.85

def calculate_CT(x):
    # Constant 
    Ct = 12

    ####

    temp = (x - 100) // 50
    final = Ct + (temp * 2)
    return final

def decide_engine(x):
    if x >= 300:
        return 2500
    else:
        return 2000

def calculate_fuel(x):
    # first check if dual or quad engine 

    checker = False

    if x >= 300:
        checker = True
    else:
        pass

    # calculate fuel load based on best time cost

    pe = 20 #each engine consumes fuel at 20kg/min
    fuel = 0

    if checker:
        fuel = (4*pe) * Tb
    else:
        fuel = (2*pe) * Tb

    return fuel

def main():
    #get input
    p = int(input("Enter the passenger capacity: "))
    flights = math.ceil(3300 / p)
    if p not in range(100, 451, 50):
        print("Capacity must be 100 to 450, in steps of 50.")
        return
    print("Flights needed:", flights)
    if flights > 13:
        print("Not feasible: more than 13 flights needed.")
        return
    fuel = calculate_fuel(p)
    Ct = calculate_CT(p)
    Cc = decide_engine(p)

    #calculate total cost

    cost_per_flight = (Cf * fuel) + (Ct * Tb) + Cc
    total_cost = cost_per_flight * flights
    engines = 4 if p >= 300 else 2

    print("Engine count:", engines)
    print(f"Fuel cost per flight: ${Cf * fuel:.2f}")
    print(f"Time cost per flight: ${Ct * Tb:.2f}")
    print(f"Fixed cost per flight: ${Cc:.2f}")

    print(f"Cost per flight: ${cost_per_flight:.2f}")
    print(f"Total cost: ${total_cost:.2f}")

while(True):
    main()

