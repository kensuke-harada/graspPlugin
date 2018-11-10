import yaml
import sys
from collections import OrderedDict


def represent_odict(dumper, instance):
    return dumper.represent_mapping('tag:yaml.org,2002:map', instance.items())

yaml.add_representer(OrderedDict, represent_odict)

##file reading
print('Please enter an input file name.')
filename = raw_input()
with open(filename) as file:

    ##Ordered Dictionary
    yaml.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    lambda loader, node: OrderedDict(loader.construct_pairs(node)))

    obj = yaml.load(file)

    for time in range(20):

        ##time
        sys.stdout.write('time = ')
        time = int(input())
        
        ##translation
        sys.stdout.write('translation = ')
        print obj['sequence'][time]['pose']['translation']
        sys.stdout.write('x = ')
        x = input()
        sys.stdout.write('y = ')
        y = input()
        sys.stdout.write('z = ')
        z = input()

        obj['sequence'][time]['pose']['translation'] = [x,y,z]

        sys.stdout.write('updated translation = ')
        print obj['sequence'][time]['pose']['translation']
        
        ##rotation
        sys.stdout.write('rotation = ')
        print obj['sequence'][time]['pose']['rotation']
        sys.stdout.write('rotation = ')
        x1, x2, x3 = map(float, raw_input().split())
        sys.stdout.write('           ')
        y1, y2, y3 = map(float, raw_input().split())
        sys.stdout.write('           ')
        z1, z2, z3 = map(float, raw_input().split())
                
        obj['sequence'][time]['pose']['rotation'] = [x1,x2,x3,y1,y2,y3,z1,z2,z3]

        sys.stdout.write('updated rotation = ')
        print obj['sequence'][time]['pose']['rotation']
                
        ##continue
        print('continue = 0, end = 1')
        c = int(input())
        if c == 1:
          break
    
##file writing
print('Please enter an output file name.')
output_filename = raw_input()
with open(output_filename, 'w') as file:

    yaml.dump(obj, file)

