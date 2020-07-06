# -*- coding: utf-8 -*-
"""
This module is used to generate an Acme description from a set of nodes parsed
from a launch file.

The main class provided by this module is :class:`AcmeGenerator`
"""
from typing import Dict, Iterator, Optional, Tuple, List, Sequence
import os
import subprocess
from subprocess import PIPE
import json
import tempfile

from loguru import logger

from ..interpreter import NodeSummary

# Constants for Acme generation
TOPIC_CONNECTOR = """   connector {conn_name} : TopicConnectorT = new TopicConnectorT extended with {{
    {roles}
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """
SERVICE_CONNECTOR = """  connector {conn_name} : ServiceConnT = new ServiceConnT extended with {{
    {roles}
    }};
   """

ACTION_CONNECTOR = """  connector {conn_name} : ActionServerConnT = new ActionServerConnT extended with {{
    {roles}
    }};
   """

NODE_COMPONENT = """   component {comp_name} : ROSNodeCompT = new ROSNodeCompT extended with {{
        {ports}
        property name = "{node_name}";
        property launch_file = "{filename}";
    }};
    """
NODE_PLACEHOLDER_COMPONENT = """   component {comp_name} : ROSNodeCompT, PlaceholderT = new ROSNodeCompT, PlaceholderT extended with {{
        {ports}
        property name = "{node_name}";
        property placeholder = true;
        property launch_file = "{filename}";
    }};
"""

SUBSCRIBER_ROLE = """    role {role_name} : ROSTopicSubscriberRoleT = new ROSTopicSubscriberRoleT;
    """

ADVERTISER_ROLE = """     role {role_name} : ROSTopicAdvertiserRoleT = new ROSTopicAdvertiserRoleT;
    """

PROVIDER_ROLE = """     role {role_name} : ROSServiceCallRoleT = new ROSServiceCallRoleT;
    """

CLIENT_ROLE = """       role {role_name} : ROSServiceProviderRoleT = new ROSServiceProviderRoleT;
    """

ACTION_CLIENT_ROLE = """      role {role_name} : ROSActionCallerRoleT = new ROSActionCallRoleT;
    """

ACTION_SERVER_ROLE = """      role {role_name} : ROSActionResponderRoleT = new ROSActionResponderRoleT;
    """

TOPIC_PORT = """     port {port_name} : ${port_type} = new ${port_type} extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """

SUBSCRIBER_PORT = """     port {port_name} : TopicSubscribePortT = new TopicSubscribePortT extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """
ADVERTISER_PORT = """     port {port_name} : TopicAdvertisePortT = new TopicAdvertisePortT extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """

PROVIDER_PORT = """     port {port_name} : ServiceProviderPortT = new ServiceProviderPortT extended with {{
        property svc_type : string = "{svc_type}";
        property name : string = "{service}";
        property args : string = "";
    }};
    """

REQUIRER_PORT = """     port {port_name} : ServiceClientPortT = new ServiceClientPortT extended with {{
        property svc_type : string = "{svc_type}";
        property persistency : boolean = {persistence}
    }};
    """

ACTION_CLIENT_PORT = """    port {port_name} : ActionClientPortT = new ActionClientPortT extended with {{
        property action_type : string = "{action_type}";
    }};
    """

ACTION_SERVER_PORT = """    port {port_name}: ActionServerPortT = new ActionServerPortT extended with {{
        property action_type : string = "{action_type}";
    }};
    """


def update_service_conn(conns, service, port_qualified, is_provider) -> None:
    s = {}
    if service in conns:
        s = conns[service]
    else:
        s = {"name": service, 'callers': set(), 'providers': set()}
        # s.name = service
        # s.callers= {}
        # s.providers = {}
        conns[service] = s
    if is_provider:
        s['providers'].add(port_qualified)
    else:
        s['callers'].add(port_qualified)


def update_action_conn(conns, action, port_qualified, is_server) -> None:
    a = {}
    if action in conns:
        a = conns[action]
    else:
        a = {'name': action, "clients": set(), 'servers': set()}
        conns[action] = a
    if is_server:
        a['servers'].add(port_qualified)
    else:
        a['clients'].add(port_qualified)


class AcmeGenerator:
    def __init__(self,
                 nodes: Iterator[NodeSummary],
                 acme_file: str,
                 jar: Optional[str]
                 ) -> None:
        self.__nodes: Sequence[NodeSummary] = list(nodes)
        self.__acme_file = acme_file
        self.__generate_dangling_connectors = False
        self.__acme_jar = jar if jar is not None else 'lib/acme.standalone-ros.jar'

    def get_components_and_connectors(self) \
            -> Tuple[List[NodeSummary], Dict[str, dict], Dict[str, dict], Dict[str, dict]]:
        components: List[NodeSummary] = []
        topics: Dict[str, dict] = {}
        services: Dict[str, dict] = {}
        actions: Dict[str, dict] = {}

        for node in self.__nodes:
            for pTopic, pType in node.pubs:
                topic = {}
                if pTopic in topics:
                    topic = topics[pTopic]
                else:
                    topic = {'details': {'name': pTopic, 'format': pType}, "pubs": [], "subs": []}
                    topics[pTopic] = topic
                topic["pubs"].append(node.name)
            for sTopic, sType in node.subs:
                topic = {}
                if sTopic in topics:
                    topic = topics[sTopic]
                else:
                    topic = {'details': {'name': sTopic, 'format': sType}, "pubs": [], "subs": []}
                    topics[sTopic] = topic
                topic["subs"].append(node.name)
            for sName, sType in node.provides:
                service = {}
                if sName in services:
                    service = services[sName]
                else:
                    service = {'details': {'name': sName, 'format': sType}, "provs": [], "reqs": []}
                    services[sName] = service
                service["provs"].append(node.name)
            # for call in node()['requires']:
            #     service={}
            #     if call["name"] in services:
            #         service = services[call["name"]]
            #     else:
            #         service = {'details' : call, "provs": [], "reqs" : []}
            #         services[call["name"]] = service
            #     service["reqs"].append(node()["name"])
            for aName, aType in node.action_servers:
                action = {}
                if aName in actions:
                    action = actions[aName]
                else:
                    action = {'details': {'name': aName, 'type': aType}, "servers": [], "clients": []}
                    actions[aName] = action
                action["servers"].append(node.name)
            for aName, aType in node.action_clients:
                action = {}
                if aName in actions:
                    action = actions[aName]
                else:
                    action = {'details': {'name': aName, 'format': aType}, "servers": [], "clients": []}
                    actions[aName] = action
                action["clients"].append(node.name)
            components.append(node)
        return components, topics, services, actions

    @staticmethod
    def to_acme_name(name: str) -> str:
        return name.replace("/", "_")

    def generate_acme(self) -> str:
        components, topics, services, actions = \
            self.get_components_and_connectors()

        system_name = "RobotSystem" if self.__acme_file is None else '_'.join(self.__acme_file.split(".")[:-1])
        # system_name = os.path.basename(os.path.normpath(self.__launch_files)).split('.')[0]

        acme = f"import families/ROSFam.acme;\nsystem {system_name} : ROSFam = new ROSFam extended with {{\n"
        attachments: List[str] = []
        component_strs: List[str] = []
        service_conns: Dict[str, dict] = {}
        action_conns: Dict[str, dict] = {}
        attachments_to_topic: Dict[str, List[str]] = {}
        ATTACHMENT = "  attachment {comp}.{port} to {conn}.{role};"
        SERVICE_ATTACHMENT = "  attachment {qualified_port} to {conn}.{role};"
        for c in components:
            ports = []
            comp_name = self.to_acme_name(c.name)

            for (pTopic, pType) in c.pubs:
                if pTopic not in attachments_to_topic:
                    attachments_to_topic[pTopic] = []
                pname = f'{self.to_acme_name(pTopic)}_pub'
                port = ADVERTISER_PORT.format(port_name=pname, msg_type=pType, topic=pTopic)
                ports.append(port)
                attachments_to_topic[pTopic].append(
                    ATTACHMENT.format(comp=comp_name,
                                      port=pname,
                                      conn=f"{AcmeGenerator.to_acme_name(pTopic)}_conn",
                                      role=f"{comp_name}_pub"))
            for name, fmt in c.subs:
                if name not in attachments_to_topic:
                    attachments_to_topic[name] = []
                pname = f"{AcmeGenerator.to_acme_name(name)}_sub"
                port = SUBSCRIBER_PORT.format(port_name=pname, msg_type=fmt, topic=name)
                ports.append(port)
                attachments_to_topic[name].append(
                    ATTACHMENT.format(comp=comp_name,
                                      port=pname,
                                      conn=f"{self.to_acme_name(name)}_conn",
                                      role=f"{comp_name}_sub"))
            for name, fmt in c.provides:
                pname = f"{AcmeGenerator.to_acme_name(name)}_svc"
                port = PROVIDER_PORT.format(port_name=pname, svc_type=fmt, service=name)
                ports.append(port)
                update_service_conn(service_conns, name, f"{comp_name}.{pname}", True)
            # Left here for when we do calls
            # for s in c['calls']:
            #     pname=self.to_acme_name(s['name']) + "_call"
            #     port = REQUIRER_PORT.format(port_name=pname, svc_type=s['format'], service=s['name'])
            #     ports.append(port)
            #     self.update_service_conn(service_conns,s['name'], "%s.%s" %(comp_name, pname), False)

            for name, fmt in c.action_servers:
                pname = f"{AcmeGenerator.to_acme_name(name)}_srvr"
                port = ACTION_SERVER_PORT.format(port_name=pname,
                                                 action_type=name)
                ports.append(port)
                update_action_conn(action_conns,
                                   name,
                                   f"{comp_name}.{pname}",
                                   True)

            for name, fmt in c.action_clients:
                pname = f"{AcmeGenerator.to_acme_name(name)}_cli"
                port = ACTION_CLIENT_PORT.format(port_name=pname,
                                                 action_type=name)
                ports.append(port)
                update_action_conn(action_conns,
                                   name,
                                   f"{comp_name}.{pname}",
                                   False)
            component_template: str = NODE_COMPONENT if not c.placeholder else NODE_PLACEHOLDER_COMPONENT
            comp = component_template.format(comp_name=comp_name,
                                             ports='\n'.join(ports),
                                             node_name=c.name,
                                             filename=c.filename)
            component_strs.append(comp)
        acme = acme + "\n".join(component_strs)

        connector_strs = []
        for t in topics.keys():
            if len(topics[t]["pubs"]) + len(topics[t]["subs"]) > 1:
                roles = []
                for p in topics[t]["pubs"]:
                    rname = f"{p}_pub"
                    role = ADVERTISER_ROLE.format(role_name=rname)
                    roles.append(role)
                for s in topics[t]["subs"]:
                    rname = f"{s}_sub"
                    role = SUBSCRIBER_ROLE.format(role_name=rname)
                    roles.append(role)
                cname = AcmeGenerator.to_acme_name(topics[t]["details"]['name']) + "_conn"
                conn = TOPIC_CONNECTOR.format(conn_name=cname, roles="\n".join(roles),
                                              msg_type=topics[t]["details"]['format'],
                                              topic=topics[t]["details"]['name'])
                connector_strs.append(conn)

                for a in attachments_to_topic[t]:
                    attachments.append(a)

        for s in service_conns.keys():
            # Only create a connector for services that are connected
            if self.__generate_dangling_connectors or (
                    len(service_conns[s]['providers']) != 0 and len(service_conns[s]['callers']) != 0):
                roles = []
                cname = f"{AcmeGenerator.to_acme_name(s)}_conn"
                for p in service_conns[s]['providers']:
                    rname = AcmeGenerator.to_acme_name(p)
                    role = PROVIDER_ROLE.format(role_name=rname)
                    roles.append(role)
                    attachments.append(SERVICE_ATTACHMENT.format(qualitifed_port=p, conn=cname, role=rname))
                for p in service_conns[s]['callers']:
                    rname = AcmeGenerator.to_acme_name(p)
                    role = CLIENT_ROLE.format(role_name=rname)
                    roles.append(f"{role}\n")
                    attachments.append(SERVICE_ATTACHMENT.format(qualitifed_port=p,
                                                                 conn=cname,
                                                                 role=rname))
                connector_strs.append(SERVICE_CONNECTOR.format(conn_name=cname, roles="\n".join(roles)))

        for a in action_conns.keys():
            # only create a connector for actions that are connected
            if self.__generate_dangling_connectors or (
                    len(action_conns[a]['servers']) != 0 and len(action_conns[a]['clients']) != 0):
                roles = []
                cname = f"{AcmeGenerator.to_acme_name(a)}_conn"
                for cl in action_conns[a]['clients']:
                    rname = AcmeGenerator.to_acme_name(cl)
                    role = ACTION_CLIENT_ROLE.format(role_name=rname)
                    roles.append(role)
                    attachments.append(SERVICE_ATTACHMENT.format(qualified_port=c, conn=cname, role=rname))
                for s in action_conns[a]['servers']:
                    rname = AcmeGenerator.to_acme_name(s)
                    role = ACTION_SERVER_ROLE.format(role_name=rname)
                    roles.append(role)
                    attachments.append(SERVICE_ATTACHMENT.format(qualified_port=c, conn=cname, role=rname))
                connector_strs.append(ACTION_CONNECTOR.format(conn_name=cname, roles="\n".join(roles)))
        acme = acme + "\n".join(connector_strs)
        acme = acme + "\n".join(attachments) + "}"
        self.generate_acme_file(acme)
        return acme

    @staticmethod
    def check_acme_file(filename: str) -> Tuple[bytes, bytes]:
        process = subprocess.Popen(list(['java', '-jar', 'lib/acme.standalone-ros.jar', filename]))
        (output, err) = process.communicate()
        process.wait()
        return output, err

    def check_acme_string(self, acme: str) -> Tuple[bytes, bytes]:
        f, filename = tempfile.mkstemp()
        try:
            with open(filename, 'w') as fi:
                fi.write(acme)
                fi.close()
            return self.check_acme_file(filename)
        finally:
            os.unlink(filename)

    def generate_acme_file(self, acme: str):
        if self.__acme_file is not None:
            logger.info(f"Writing Acme to {self.__acme_file}")
            with open(self.__acme_file, 'w') as f:
                f.write(acme)

    def check_acme(self):
        self._check_acme(self.__acme_file)

    def _check_acme(self, acme_file: str):
        (_, jf) = tempfile.mkstemp(suffix=".json")
        try:
            logger.debug("Running Acme checker")
            print("Checking architecture...")
            run = subprocess.run(list(["java", "-jar", self.__acme_jar, "-j", jf, acme_file]), stdout=PIPE, stderr=PIPE)
            if run.returncode == 0:
                logger.debug("Checking ran successfully")
                logger.debug(run.stdout)
                with open(jf, 'r') as j:
                    checks = json.load(j)
                if len(checks["errors"]) == 0:
                    print("Robot architecture has no errors")
                else:
                    print("The following problems were found with the robot architecture:")
                    for e in checks["errors"]:
                        if 'causes' not in e.keys() or len(e['causes']) == 0:
                            print(f"    {e['error']}")
                        else:
                            for c in e["causes"]:
                                print(f"    {c}")

            else:
                logger.error("Could not run the checker")
                logger.error(run.stderr)
        finally:
            os.remove(jf)
