import xmlrpclib


server = xmlrpclib.ServerProxy('http://localhost:8001')

#server.register_introspection_functions()
print server.get_angle('HeadYaw')
print server.system.listMethods()
