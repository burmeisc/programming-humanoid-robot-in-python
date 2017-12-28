from SimpleXMLRPCServer import SimpleXMLRPCServer


print "start Server.."
server = SimpleXMLRPCServer(("localhost", 8001))
print "server startet: %s"%str(server)
server.register_introspection_functions()
server.register_function(pow)

def get_angle(joint_angle):
    return "function get_angle called, with parameter %s called"%joint_angle
    


server.register_function(get_angle)

server.serve_forever()
