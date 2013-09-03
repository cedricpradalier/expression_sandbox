#include <stdlib.h>
#include <stdio.h>

#include "expressions/Expressions.h"

int main()
{
    Expression x("x"), y("y");
    Expression f = (x + cos(x * y) - y) / sin(y);
    
    SubstitutionMap map;
    map.insert(SubstitutionMap::value_type("x", 1.0));
    map.insert(SubstitutionMap::value_type("y", 5.0));

    double value = f(map);

    printf("Expression f: %s\n",f.toString().c_str());
    printf("Value map:\n");
    for (SubstitutionMap::const_iterator it = map.begin();it != map.end(); it++) {
        printf("%-16s: %g\n",it->first.c_str(),it->second);
    }
    printf("Value: %g\n",value);

    VariableId ids = f.getVariables();
    // Not really implemented
    printf("Jacobian (for x=y): %s\n",f.jacobian(ids).toString().c_str());

    return 0;
}

