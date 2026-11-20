import math
# REVIEW NOTE: Per-flight cost = Cf * fuel_mass + Ct * Tb + Cc (lecture p.7 example).
# calculate_fuel returns kg, not kg/min: do not multiply fuel_mass by Tb again.
# Total cost multiplies this unrounded per-flight cost by the required flights.
## constant term: T(best) = 58.7875 minutes
## Based on scenario 1
# REVIEW TODO (not applied): Lecture p.26 also asks for an aircraft name.
# Keep the original terminal format; add the name in the report, or approve new output.
# Tb below is a fixed rounded input, not a live import from the route planner.
## C(f) = 0.85 $/kg

Tb = 58.7875
Cf = 0.85

def calculate_CT(x):
    # REVIEW NOTE: This model interprets the $12/min base as the 100-seat design.
    # Each extra 50 seats adds $2/min; preserve the existing base interpretation.
    Ct = 12

    temp = (x - 100) // 50
    final = Ct + (temp * 2)
    return final

def decide_engine(x):
    # REVIEW NOTE: Legacy name retained; this returns fixed cost, not engine count.
    # Lecture p.26: 300+ seats require four engines and $2500 per flight.
    if x >= 300:
        return 2500
    else:
        return 2000

def calculate_fuel(x):
    # REVIEW CHANGE: Derive engine count directly, removing redundant flag branches.
    # Preserve (engines * 20) * Tb, including the original multiplication order.
    engines = 4 if x >= 300 else 2
    pe = 20  # Lecture p.26: each engine consumes 20 kg/min during cruise.
    fuel = (engines * pe) * Tb

    return fuel

def main():
    #get input
    p = int(input("Enter the passenger capacity: "))
    # REVIEW TODO (not applied): Validate before division and handle non-integers.
    # Moving validation would change legacy zero/invalid-input errors, so defer it.
    flights = math.ceil(3300 / p)
    if p not in range(100, 451, 50):
        print("Capacity must be 100 to 450, in steps of 50.")
        return
    print("Flights needed:", flights)
    # REVIEW NOTE: 100/150/200/250 seats need 33/22/17/14 flights: all infeasible.
    # Thus 300 seats is the smallest permitted feasible option, requiring four engines.
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

# REVIEW NOTE: Preserve repeated prompting; stop with Ctrl+C as in the original.
while(True):
    main()


