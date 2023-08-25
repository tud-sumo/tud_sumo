import os, requests, csv, xml.etree.ElementTree as ET
from tud_sumo import Simulation

tu_delft_bbox = (52.004668, 51.991784, 4.383724, 4.365496)
all_delft_bbox = (52.024396, 51.981332, 4.398933, 4.305552)
exe_bbox = (50.737069, 50.704257, -3.491951, -3.559872)
london_bbox = (51.540041, 51.488085, -0.073101, -0.153815)
m25_bbox = (51.737469, 51.253273, 0.354171, -0.570450)
rotterdam_ring = (51.970420, 51.850273, 4.588479, 4.332092)
bordeaux = (44.835009, 44.831368, -0.596215, -0.603005)

def sim_from_osm(scenario_name, useragent, query, road_level=-1, include_ped_infr=False, include_buildings=False, sumocfg_vals=None, netconvert_args=None, overwrite_files=True, remove_osm=True) -> str:
    """
    Creates a .net.xml and .sumocfg file from OSM data.
    :param scenario_name:     Scenario name (ID)
    :param useragent:         String "project_name/contact_details" (required for requesting OSM data)
    :param query:             Either area bbox (N, S, E, W coors), '.osm' file, '.txt' file, or query string (use https://overpass-turbo.eu/)
    :param road_level:        Road hierarchy level (motorway = 1 => tracks = 7), defaults to all
    :param include_ped_infr:  Include pedestrian and bike infrastructure in simulation
    :param include_buildings: Include buildings in SUMO simulation
    :param sumocfg_vals:      Additional configuration settings (step-length etc.)
    :param netconvert_args:   Additional netconvert command line arguments
    :param overwrite_files:   If false, raise an error when overwriting previously saved scenarios
    :param remove_osm:        If true, remove all '.osm' files once complete
    :return str:              SUMO config filepath
    """

    headers = {
        'Connection': 'keep-alive', 'sec-ch-ua': '"Google Chrome 80"',
        'Accept': '*/*', 'Sec-Fetch-Dest': 'empty', 'User-Agent': useragent,
        'Content-Type': 'application/x-www-form-urlencoded; charset=UTF-8',
        'Origin': 'https://overpass-turbo.eu',
        'Sec-Fetch-Site': 'cross-site', 'Sec-Fetch-Mode': 'cors',
        'Referer': 'https://overpass-turbo.eu/', 'Accept-Language': '', 'dnt': '1',
    }

    bbox = False
    if not (isinstance(query, str) and query.endswith('.osm') and os.path.isfile(query)):
        if isinstance(query, str):
            if query.endswith(".txt") and os.path.exists(query):
                with open(query, 'r') as query_file:
                    query_str = ''.join(query_file.readlines())
            else: query_str = query
        elif (isinstance(query, list) or isinstance(query, tuple)) and False not in [isinstance(val, float) for val in query]:
            n, s, e, w = query
            road_levels = [['motorway', 'motorway_link'], ['trunk', 'trunk_link'], ['primary', 'primary_link'], ['secondary', 'secondary_link'],
                        ['tertiary', 'tertiary_link'], ['unclassified', 'residential'], ['service', 'track']]
            ped_road_types = ['footway', 'bridleway', 'steps', 'corridor', 'path', 'cycleway', 'living_street', 'pedestrian']
            query_str = """way({0},{1},{2},{3}) ['highway']['highway' !~ 'raceway']['highway' !~ 'proposed']
                            ['highway' !~ 'construction']['highway' !~ 'elevator']
                            ['access' !~ 'private']['access' !~ 'no']['area' !~ 'yes']""".format(s, w, n, e)
            
            if road_level != -1 or not include_ped_infr:
                types_str = ""

                if include_ped_infr:
                    types_str = "|".join(ped_road_types)

                if not include_ped_infr:
                    for ped_road_type in ped_road_types:
                        query_str += "['highway' !~ '{0}']".format(ped_road_type)

                if road_level != -1: 
                    
                    for level in road_levels[:road_level]:
                        for r_type in level:
                            if types_str == "": types_str += r_type
                            else: types_str = types_str + '|' + r_type

                if types_str != "": query_str += "['highway' ~ '"+types_str+"']"
            
            query_str += """;(._;>;);out;"""

            bbox = True
        else: raise TypeError("sim_from_osm: Invalid query (must be [str (query|filepath) | list (bbox)], not '"+type(query).__name__+"')")

        data = { 'data': query_str }
        response = requests.post('https://overpass-api.de/api/interpreter', headers=headers, data=data)

    if not os.path.isdir('scenarios'): os.makedirs('scenarios')
    if not os.path.isdir('scenarios/'+scenario_name): os.makedirs('scenarios/'+scenario_name)
    else:
        if not overwrite_files: raise FileExistsError("sim_from_osm: Scenario file '{0}' already exists and cannot be overwritten.".format(scenario_name))

    sumocfg_file = 'scenarios/'+scenario_name+'/'+scenario_name+'.sumocfg'
    osm_file = 'scenarios/'+scenario_name+'/'+scenario_name+'.osm'
    net_file = 'scenarios/'+scenario_name+'/'+scenario_name+'.net.xml'
    building_file = 'scenarios/'+scenario_name+'/buildings.poi.xml'

    with open(osm_file, 'w') as f:
        f.write(response.text)

    if bbox and include_buildings:
        query_str = """way({0},{1},{2},{3})['building'];(._;>;);out;""".format(s, w, n, e)
        data = { 'data': query_str }
        response = requests.post('https://overpass-api.de/api/interpreter', headers=headers, data=data)

        building_file = 'scenarios/'+scenario_name+'/buildings.osm'
        with open(building_file, 'w') as f:
            f.write(response.text)

        os.system("polyconvert --osm-files {0} -o scenarios/{1}/buildings.poi.xml".format(building_file, scenario_name))

    if netconvert_args == None: netconvert_args = []
    if os.path.exists(building_file): netconvert_args += ["--polygon-files", "scenarios/"+scenario_name+"/buildings.poi.xml"]

    netconvert_cmd = "netconvert --osm-files {0} --output-file {1}".format(osm_file, net_file, ''.join(netconvert_args))
    os.system(netconvert_cmd)

    inputs = ['        <net-file value="{0}"/>\n'.format(net_file.split('/')[-1])]
    if include_buildings and os.path.exists('scenarios/'+scenario_name+'/buildings.poi.xml'):
        inputs.append('        <additional-files value="buildings.poi.xml"/>\n')

    sumocfg = ['<?xml version="1.0" encoding="UTF-8"?>\n\n','<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">\n\n', '    <input>\n']
    sumocfg = sumocfg + inputs + ['    </input>\n']

    for setting, value in sumocfg_vals.items():
        sumocfg.append('    <{0} value="{1}" />\n\n'.format(setting, value))

    sumocfg.append('</configuration>')
    
    with open(sumocfg_file, 'w') as f:
        f.writelines(sumocfg)

    if remove_osm:
        if os.path.exists(osm_file): os.remove(osm_file)
        if os.path.exists(building_file): os.remove(building_file)

    return sumocfg_file

def add_sim_demand(scenario_name, od_file, start_time, end_time, vtype_id='cars', vtype_params={}):
    """
    Create simulation routes file and link to scenario '.sumocfg'. All demand/vtype data is appended to existing route files.
    :param scenario_name: Pre-existing scenario ID
    :param od_file:       Filepath to OD matrix (.csv)
    :param start_time:    Demand profile start time
    :param end_time:      Demand profile end time
    :param vtype_id:      Demand profile vehicle type (added as vType if unknown)
    :param vtype_params:  vType parameters (default values for 'cars' and 'bikes' added)
    """

    if 'cars' not in vtype_params.keys(): vtype_params['cars'] = {'color': 'red'}
    if 'bikes' not in vtype_params.keys(): vtype_params['cars'] = {'length': '1.60', 'minGap': '0.5', 'vClass': 'bicycle', 'color': 'green'}
    
    if not isinstance(od_file, str) or not od_file.endswith('.csv'): raise ValueError("add_sim_demand: Invalid OD filename '{0}'".format(od_file))
    elif not os.path.exists(od_file): raise FileNotFoundError("add_sim_demand: OD file '{0}' does not exist.".format(od_file))

    demand_file = 'scenarios/'+scenario_name+'/'+scenario_name+'.rou.xml'
    if not os.path.exists(demand_file):
        demand = ['<?xml version="1.0" encoding="UTF-8"?>\n\n<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n\n</routes>']
        with open(demand_file, 'w') as f:
            f.writelines(demand)

    demand_tree = ET.parse(demand_file)
    root = demand_tree.getroot()

    add_vtype = False
    if 'vType' not in [elem.tag for elem in root.iter()]: add_vtype = True
    else:
        curr_vtypes = [f_vtype.attrib['id'] for f_vtype in root.iter('vType')]
        if vtype_id not in curr_vtypes: add_vtype = True

    if add_vtype:
        vtype = ET.Element('vType')
        vtype.set('id', vtype_id)
        if vtype_id in vtype_params.keys():
            for attrib, val in vtype_params[vtype_id].items():
                vtype.set(attrib, val)
        root.append(vtype)

    with open(od_file, 'r') as f:
        od_reader = csv.reader(f)

        locs = None
        flow_ids = [int(f_vtype.attrib['id'].split('_')[-1]) for f_vtype in root.iter('flow')]
        flow_id = 1 if len(flow_ids) == 0 else max(flow_ids) + 1
        for row in od_reader:
            if locs == None:
                locs = row[1:]
            else:
                a = row[0]
                for i, b in enumerate(locs):
                    flow = ET.Element('flow')
                    flow.set('id', 'flow_'+str(flow_id))
                    flow.set('type', vtype_id)
                    flow.set('begin', str(float(start_time)))
                    flow.set('from', str(a))
                    flow.set('to', str(b))
                    flow.set('end', str(float(end_time)))
                    flow.set('number', row[1 + i])
                    flow_id += 1

                    root.append(flow)

    ET.indent(demand_tree, space="    ")
    demand_tree.write(demand_file)

    sumocfg_tree = ET.parse('scenarios/'+scenario_name+'/'+scenario_name+'.sumocfg')
    root = sumocfg_tree.getroot()
    for input_elem in root.iter('input'):
        if 'route-files' not in [elem.tag for elem in input_elem.iter()]:
            route_file = ET.Element('route-files')
            route_file.set('value', scenario_name+'.rou.xml')
            input_elem.append(route_file)

            ET.indent(sumocfg_tree, space="    ")
            sumocfg_tree.write('scenarios/'+scenario_name+'/'+scenario_name+'.sumocfg')

if __name__ == "__main__":

    netconvert_args =  ["--roundabouts.guess", "--ramps.guess", "--tls.guess-signals", "--tls.discard-simple", "--tls.join"]

    cfg_file = sim_from_osm('delft', "tud_sumo/c.evans@tudelft.nl", all_delft_bbox, include_buildings=True, include_ped_infr=True, sumocfg_vals={'step-length': 0.5, "lateral-resolution": 1.6}, netconvert_args=netconvert_args)

    #add_sim_demand('exeter', 'od.csv', 0, 100, 'cars')
    
    sim = Simulation()

    sim.start(cfg_file, gui=True)

    sim_data = sim.step_through(1000)