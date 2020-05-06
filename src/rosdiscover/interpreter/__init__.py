# -*- coding: utf-8 -*-
"""
This module is used to model the architectural consequences of particular
ROS commands (e.g., launching a given :code:`.launch` file via
:code:`roslaunch`).

The main class within this module is :class:`Interpreter`, which acts as a
model evaluator / virtual machine for a ROS architecture.
"""
from typing import (Dict, Iterator, Any, Optional, Tuple, Callable, Set,
                    FrozenSet, Sequence)
import logging
import contextlib

import attr
import dockerblade
import roswire
from roswire.proxy.roslaunch.reader import LaunchFileReader

from .summary import NodeSummary
from .parameter import ParameterServer

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)

FullName = str


class NodeContext:
    def __init__(self,
                 name: str,
                 namespace: str,
                 kind: str,
                 package: str,
                 args: str,
                 remappings: Dict[str, str],
                 params: ParameterServer,
                 files: dockerblade.files.FileSystem,
                 ) -> None:
        self.__name = name
        self.__namespace = namespace
        self.__kind = kind
        self.__package = package
        self.__params = params
        self.__files = files
        self.__args = args
        self.__nodelet: bool = False
        self.__uses: Set[Tuple[str, str]] = set()
        self.__provides: Set[Tuple[str, str]] = set()
        self.__subs: Set[Tuple[str, str]] = set()
        self.__pubs: Set[Tuple[str, str]] = set()

        self.__action_servers: Set[Tuple[str, str]] = set()
        self.__action_clients: Set[Tuple[str, str]] = set()

        # The tuple is (name, dynamic) where name is the name of the parameter
        # and dynamic is whether the node reacts to updates to the parameter via reconfigure
        self.__reads: Set[Tuple[str, bool]] = set()
        self.__writes: Set[str] = set()
        self.__placeholder : bool = False

        self.__remappings: Dict[str, str] = {
            self.resolve(x): self.resolve(y)
            for (x, y) in remappings.items()
        }

    @property
    def args(self) -> str:
        return self.__args

    @property
    def fullname(self) -> str:
        ns = self.__namespace
        if ns[-1] != '/':
            ns += ' /'
        return '{}{}'.format(ns, self.__name)

    def _remap(self, name: str) -> str:
        if name in self.__remappings:
            name_new = self.__remappings[name]
            logger.info("applying remapping from [%s] to [%s]",
                        name, name_new)
            return name_new
        else:
            return name

    def summarize(self) -> NodeSummary:
        return NodeSummary(name=self.__name,
                           fullname=self.fullname,
                           namespace=self.__namespace,
                           kind=self.__kind,
                           package=self.__package,
                           nodelet=self.__nodelet,
                           reads=self.__reads,
                           writes=self.__writes,
                           pubs=self.__pubs,
                           subs=self.__subs,
                           provides=self.__provides,
                           uses=self.__uses,
                           action_servers=self.__action_servers,
                           action_clients=self.__action_clients)

    def resolve(self, name: str) -> str:
        """Resolves a given name within the context of this node.

        Returns:
            the fully qualified form of a given name.
        """
        if name[0] == '/':
            return name
        elif name[0] == '~':
            return '/{}/{}'.format(self.__name, name[1:])
        # FIXME
        else:
            return '/{}'.format(name)

    def provide(self, service: str, fmt: str) -> None:
        """Instructs the node to provide a service."""
        logger.debug("node [%s] provides service [%s] using format [%s]",
                     self.__name, service, fmt)

        service_name_full = self.resolve(service)
        service_name_full = self._remap(service_name_full)
        self.__provides.add((service_name_full, fmt))

    def use(self, service: str, fmt: str) -> None:
        """Instructs the node to use a given service."""
        logger.debug("node [%s] uses a service [%s] with format [%s]",
                     self.__name, service, fmt)

        service_name_full = self.resolve(service)
        service_name_full = self._remap(service_name_full)
        self.__uses.add((service_name_full, fmt))

    def sub(self, topic_name: str, fmt: str) -> None:
        """Subscribes the node to a given topic.

        Parameters:
            topic: the unqualified name of the topic.
            fmt: the message format used by the topic.
        """
        topic_name_full = self.resolve(topic_name)
        topic_name_full = self._remap(topic_name_full)
        logger.debug("node [%s] subscribes to topic [%s] with format [%s]",
                     self.__name, topic_name, fmt)
        self.__subs.add((topic_name_full, fmt))

    def pub(self, topic_name: str, fmt: str) -> None:
        """Instructs the node to publish to a given topic.

        Parameters:
            topic: the unqualified name of the topic.
            fmt: the message format used by the topic.
        """
        topic_name_full = self.resolve(topic_name)
        topic_name_full = self._remap(topic_name_full)
        logger.debug("node [%s] publishes to topic [%s] with format [%s]",
                     self.__name, topic_name, fmt)
        self.__pubs.add((topic_name_full, fmt))

    def read(self, param: str, default: Optional[Any] = None, dynamic: Optional[bool] = False) -> None:
        """Obtains the value of a given parameter from the parameter server."""
        logger.debug("node [%s] reads parameter [%s]",
                     self.__name, param)
        param = self.resolve(param)
        self.__reads.add((param, dynamic))
        return self.__params.get(param, default)

    def write(self, param: str, val: Any) -> None:
        logger.debug("node [%s] writes [%s] to parameter [%s]",
                     self.__name, val, param)
        param = self.resolve(param)
        self.__writes.add(param)

    def read_file(self, fn: str) -> str:
        """Reads the contents of a text file."""
        return self.__files.read(fn)

    def action_server(self, ns: str, fmt: str) -> None:
        """Creates a new action server.

        Parameters:
            ns: the namespace of the action server.
            fmt: the name of the action format used by the server.
        """
        logger.debug("node [%s] provides action server [%s] with format [%s]",
                     self.__name, ns, fmt)

        ns = self.resolve(ns)
        self.__action_servers.add((ns, fmt))

        self.sub('{}/goal'.format(ns), '{}Goal'.format(fmt))
        self.sub('{}/cancel'.format(ns), 'actionlib_msgs/GoalID')
        self.pub('{}/status'.format(ns), 'actionlib_msgs/GoalStatusArray')
        self.pub('{}/feedback'.format(ns), '{}Feedback'.format(fmt))
        self.pub('{}/result'.format(ns), '{}Result'.format(fmt))

    def action_client(self, ns: str, fmt: str) -> None:
        """Creates a new action client.
        Parameters:
            ns: the namespace of the corresponding action server.
            fmt: the name of the action format used by the server.
        """
        logger.debug("node [%s] provides action client [%s] with format [%s]",
                     self.__name, ns, fmt)

        ns = self.resolve(ns)
        self.__action_clients.add((ns, fmt))

        self.pub('{}/goal'.format(ns), '{}Goal'.format(fmt))
        self.pub('{}/cancel'.format(ns), 'actionlib_msgs/GoalID')
        self.sub('{}/status'.format(ns), 'actionlib_msgs/GoalStatusArray')
        self.sub('{}/feedback'.format(ns), '{}Feedback'.format(fmt))
        self.sub('{}/result'.format(ns), '{}Result'.format(fmt))

    def mark_nodelet(self):
        self.__nodelet = True

    def mark_placeholder(self):
        self.__placeholder = True

class Model:
    """Models the architectural interactions of a node type."""
    _models: Dict[Tuple[str, str], 'Model'] = {}

    @staticmethod
    def register(package: str,
                 name: str,
                 definition: Callable[[NodeContext], None]
                 ) -> None:
        key = (package, name)
        models = Model._models
        if key in models:
            m = "model [{}] already registered for package [{}]"
            m.format(name, package)
            raise Exception(m)
        models[key] = Model(package, name, definition)
        logger.debug("registered model [%s] for package [%s]",
                     name, package)

    @staticmethod
    def find(package: str, name: str) -> 'Model':
        try:
            return Model._models[(package, name)]
        except Exception:
            m = "failed to find model for node type [{}] in package [{}]"
            m = m.format(name, package)
            logger.warning(m)
            ph = Model._models[('PLACEHOLDER', 'PLACEHOLDER')]
            ph.__package = package
            ph.__name = name
            return ph

    def __init__(self,
                 package,       # type: str
                 name,          # type: str
                 definition     # type: Callable[[NodeContext], None]
                 ):             # type: (...) -> None
        self.__package = package
        self.__name = name
        self.__definition = definition

    def eval(self, context: NodeContext) -> None:
        return self.__definition(context)


def model(package: str, name: str) -> Any:
    def register(m: Callable[[NodeContext], None]) -> Any:
        Model.register(package, name, m)
        return m
    return register


class Interpreter:
    @staticmethod
    @contextlib.contextmanager
    def for_image(image: str,
                  sources: Sequence[str]
                  ) -> Iterator['Interpreter']:
        """Constructs an interpreter for a given Docker image."""
        rsw = roswire.ROSWire()  # TODO don't maintain multiple instances
        with rsw.launch(image, sources) as app:
    def __init__(self,
                 files: dockerblade.files.FileSystem,
                 shell: dockerblade.shell.Shell
                 ) -> None:
        self.__files = files
        self.__shell = shell
        self.__params = ParameterServer()
        self.__nodes: Set[NodeSummary] = set()

    @property
    def parameters(self) -> ParameterServer:
        """The simulated parameter server for this interpreter."""
        return self.__params

    @property
    def nodes(self) -> Iterator[NodeSummary]:
        """Returns an iterator of summaries for each ROS node."""
        yield from self.__nodes

    def launch(self, fn: str) -> None:
        """Simulates the effects of `roslaunch` using a given launch file."""
        # NOTE this method also supports command-line arguments
        reader = LaunchFileReader(shell=self.__shell,
                                  files=self.__files)

        # Workaround:
        # https://answers.ros.org/question/299232/roslaunch-python-arg-substitution-finds-wrong-package-folder-path/
        # https://github.com/ros/ros_comm/blob/e96c407c64e1c17b0dd2bb85b67f388380527097/tools/roslaunch/src/roslaunch/substitution_args.py#L141L145
        sed_command = 's#$(find xacro)/xacro #$(find xacro)/xacro.py #g'
        sed_command = f'sed -i "{sed_command}" {fn}'
        self.__shell.check_output(sed_command)

        config = reader.read(fn)

        for key, value in config.params.items():
            self.__params[key] = value

        for node in config.nodes:
            logger.debug("launching node: %s", node.name)
            try:
                args = node.args or ''
                remappings = {old: new for (old, new) in node.remappings}
                self.load(pkg=node.package,
                          nodetype=node.typ,
                          name=node.name,
                          namespace=node.namespace,  # FIXME
                          remappings=remappings,
                          args=args)
            except Exception:
                logger.exception("failed to launch node: %s", node.name)
                raise

    def create_nodelet_manager(self, name: str) -> None:
        """Creates a nodelet manager with a given name."""
        logger.info('launched nodelet manager: %s', name)

    def load_nodelet(self,
                     pkg: str,
                     nodetype: str,
                     name: str,
                     namespace: str,
                     remappings: Dict[str, str],
                     manager: Optional[str] = None
                     ) -> None:
        """Loads a nodelet using the provided instructions.

        Parameters:
            pkg: the name of the package to which the nodelet belongs.
            nodetype: the name of the type of nodelet that should be loaded.
            name: the name that should be assigned to the nodelet.
            namespace: the namespace into which the nodelet should be loaded.
            remappings: a dictionary of name remappings that should be applied
                to this nodelet, where keys correspond to old names and values
                correspond to new names.
            manager: the name of the manager, if any, for this nodelet. If
                this nodelet is standalone, :code:`manager` should be set to
                :code:`None`.

        Raises:
            Exception: if there is no model for the given nodelet type.
        """
        if manager:
            logger.info('launching nodelet [%s] inside manager [%s]',
                        name, manager)
        else:
            logger.info('launching standalone nodelet [%s]', name)
        return self.load(pkg, nodetype, name, namespace, remappings, '')

    def load(self,
             pkg: str,
             nodetype: str,
             name: str,
             namespace: str,
             remappings: Dict[str, str],
             args: str
             ) -> None:
        """Loads a node using the provided instructions.

        Parameters
        ----------
        pkg: str
            the name of the package to which the node belongs.
        nodetype: str
            the name of the type of node that should be loaded.
        name: str
            the name that should be assigned to the node.
        namespace: str
            the namespace into which the node should be loaded.
        remappings: Dict[str, str]
            a dictionary of name remappings that should be applied
            to this node, where keys correspond to old names and values
            correspond to new names.
        args: str
            a string containing command-line arguments to the node.

        Raises
        ------
        Exception
            if there is no model for the given node type.
        """
        args = args.strip()
        if nodetype == 'nodelet':
            if args == 'manager':
                return self.create_nodelet_manager(name)
            elif args.startswith('standalone '):
                pkg_and_nodetype = args.partition(' ')[2]
                pkg, _, nodetype = pkg_and_nodetype.partition('/')
                return self.load_nodelet(pkg, nodetype, name, namespace, remappings)
            else:
                load, pkg_and_nodetype, mgr = args.split(' ')
                pkg, _, nodetype = pkg_and_nodetype.partition('/')
                return self.load_nodelet(pkg, nodetype, name, namespace, remappings, mgr)

        if remappings:
            logger.info("using remappings: %s", remappings)

        try:
            model = Model.find(pkg, nodetype)
        except Exception:
            m = "failed to find model for node type [{}] in package [{}]"
            m = m.format(nodetype, pkg)
            logger.warning(m)
            raise Exception(m)

        ctx = NodeContext(name=name,
                          namespace=namespace,
                          kind=nodetype,
                          package=pkg,
                          args=args,
                          remappings=remappings,
                          files=self.__files,
                          params=self.__params)
        model.eval(ctx)

        self.__nodes.add(ctx.summarize())
