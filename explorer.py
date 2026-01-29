import compas_tna
import compas_tna.equilibrium as eq
dir(eq)

import inspect
print(inspect.signature(eq.vertical_from_zmax))
print(eq.vertical_from_zmax.__doc__[:400])

import inspect
print(inspect.getsourcefile(eq.vertical_from_zmax))

