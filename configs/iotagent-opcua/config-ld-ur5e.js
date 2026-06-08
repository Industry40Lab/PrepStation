/* Copyright 2022 Engineering Ingegneria Informatica S.p.A. */

const config = {};

config.iota = {
    /**
     * Configures the log level. Appropriate values are: FATAL, ERROR, INFO, WARN and DEBUG.
     */
    logLevel: 'DEBUG',
    /**
     * When this flag is active, the IoTAgent will add the TimeInstant attribute to every entity created, as well
     * as a TimeInstant metadata to each attribute, with the current timestamp.
     */
    timestamp: true,
    /**
     * Context Broker configuration. Defines the connection information to the instance of the Context Broker where
     * the IoT Agent will send the device data.
     */
    contextBroker: {
        /**
         * Host where the Context Broker is located.
         */
        host: 'orion_ld',
        /**
         * Port where the Context Broker is listening.
         */
        port: '1026',
        /**
         * Version of the Context Broker (v2 or ld)
         */
        ngsiVersion: 'ld',
        /**
         * JSON LD Context
         */
        jsonLdContext: 'https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context.jsonld',
        /**
         * Used as fallbackTenant
         */
        service: 'opcua_ur5e',
        /**
         * Used as fallbackPath
         */
        subservice: '/mydemo'
    },
    /**
     * Configuration of the North Port of the IoT Agent.
     */
    server: {
        /**
         * Port where the IoT Agent will be listening for NGSI and Provisioning requests.
         */
        port: 4041
    },

     
    deviceRegistry: {
        type: 'mongodb'
    },
    /**
     * Mongo DB configuration section. This section will only be used if the deviceRegistry property has the type
     * 'mongodb'.
     */
    mongodb: {
        /**
         * Host where MongoDB is located. If the MongoDB used is a replicaSet, this property will contain a
         * comma-separated list of the instance names or IPs.
         */
        host: 'mongo_db',
        /**
         * Port where MongoDB is listening. In the case of a replicaSet, all the instances are supposed to be listening
         * in the same port.
         */
        port: '27017',
        /**
         * Name of the Mongo database that will be created to store IoT Agent data.
         */
        db: 'iotagent_opcua_myrobot'
    },
    /**
     * Types array for static configuration of services. Check documentation in the IoT Agent Library for Node.js for
     *  further details:
     *
     *      https://github.com/Engineering-Research-and-Development/iotagent-opcua#type-configuration
     */
    types: {
        
        JointType: {
            active: [
                {
                    name: "Current",
                    type: "Float"
                },
                {
                    name: "Position",
                    type: "Float"
                },
                {
                    name: "Temperature",
                    type: "Float"
                },
                {
                    name: "Velocity",
                    type: "Float"
                },
            ]
        },

        
        TcpType: {
            active: [
                {
                    name: "Force",
                    type: "array[]"
                },
                {
                    name: "Pose",
                    type: "array[]"
                },
                {
                    name: "Speed",
                    type: "array[]"
                },
               
            ]
        },
        // Robot: {
        //     active: [
        //         {
        //             name: "Joint",
        //             type: "JointType"
        //         },
              
               
        //     ]
        // },
    },

    contexts: [

        {
            id: "urn:ngsi-ld:TcpType:tcp0",
            type: "TcpType",
            mappings: [
                {
                    ocb_id: "Force",
                    opcua_id: "ns=0;i=54601",
                    object_id: "ns=0;i=54601",
                    inputArguments: []
                },
                {
                    ocb_id: "Pose",
                    opcua_id: "ns=0;i=54599",
                    object_id: "ns=0;i=54599",
                    inputArguments: []
                },
                {
                    ocb_id: "Speed",
                    opcua_id: "ns=0;i=54600",
                    object_id: "ns=0;i=54600",
                    inputArguments: []
                },
            ]
        },
        {
            id: "urn:ngsi-ld:JointType:base",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "Current",
                    opcua_id: "ns=0;i=54565",
                    object_id: "ns=0;i=54565",
                    inputArguments: []
                },
                {
                    ocb_id: "Position",
                    opcua_id: "ns=0;i=54563",
                    object_id: "ns=0;i=54563",
                    inputArguments: []
                },
                {
                    ocb_id: "Temperature",
                    opcua_id: "ns=0;i=54566",
                    object_id: "ns=0;i=54566",
                    inputArguments: []
                },
                {
                    ocb_id: "Velocity",
                    opcua_id: "ns=0;i=54564",
                    object_id: "ns=0;i=54564",
                    inputArguments: []
                },
            ]
        },
        {
            id: "urn:ngsi-ld:JointType:elbow",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "Current",
                    opcua_id: "ns=0;i=54577",
                    object_id: "ns=0;i=54577",
                    inputArguments: []
                },
                {
                    ocb_id: "Position",
                    opcua_id: "ns=0;i=54575",
                    object_id: "ns=0;i=54575",
                    inputArguments: []
                },
                {
                    ocb_id: "Temperature",
                    opcua_id: "ns=0;i=54578",
                    object_id: "ns=0;i=54578",
                    inputArguments: []
                },
                {
                    ocb_id: "Velocity",
                    opcua_id: "ns=0;i=54576",
                    object_id: "ns=0;i=54576",
                    inputArguments: []
                },
            ]
        },
        {
            id: "urn:ngsi-ld:JointType:shoulder",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "Current",
                    opcua_id: "ns=0;i=54571",
                    object_id: "ns=0;i=54571",
                    inputArguments: []
                },
                {
                    ocb_id: "Position",
                    opcua_id: "ns=0;i=54569",
                    object_id: "ns=0;i=54569",
                    inputArguments: []
                },
                {
                    ocb_id: "Temperature",
                    opcua_id: "ns=0;i=54572",
                    object_id: "ns=0;i=54572",
                    inputArguments: []
                },
                {
                    ocb_id: "Velocity",
                    opcua_id: "ns=0;i=54570",
                    object_id: "ns=0;i=54570",
                    inputArguments: []
                },
            ]
        },
        {
            id: "urn:ngsi-ld:JointType:wrist1",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "Current",
                    opcua_id: "ns=0;i=54583",
                    object_id: "ns=0;i=54583",
                    inputArguments: []
                },
                {
                    ocb_id: "Position",
                    opcua_id: "ns=0;i=54581",
                    object_id: "ns=0;i=54581",
                    inputArguments: []
                },
                {
                    ocb_id: "Temperature",
                    opcua_id: "ns=0;i=54584",
                    object_id: "ns=0;i=54584",
                    inputArguments: []
                },
                {
                    ocb_id: "Velocity",
                    opcua_id: "ns=0;i=54582",
                    object_id: "ns=0;i=54582",
                    inputArguments: []
                },
            ]
        },
        {
            id: "urn:ngsi-ld:JointType:wrist2",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "Current",
                    opcua_id: "ns=0;i=54589",
                    object_id: "ns=0;i=54589",
                    inputArguments: []
                },
                {
                    ocb_id: "Position",
                    opcua_id: "ns=0;i=54587",
                    object_id: "ns=0;i=54587",
                    inputArguments: []
                },
                {
                    ocb_id: "Temperature",
                    opcua_id: "ns=0;i=54590",
                    object_id: "ns=0;i=54590",
                    inputArguments: []
                },
                {
                    ocb_id: "Velocity",
                    opcua_id: "ns=0;i=54588",
                    object_id: "ns=0;i=54588",
                    inputArguments: []
                },
            ]
        },
        {
            id: "urn:ngsi-ld:JointType:wrist3",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "Current",
                    opcua_id: "ns=0;i=54595",
                    object_id: "ns=0;i=54595",
                    inputArguments: []
                },
                {
                    ocb_id: "Position",
                    opcua_id: "ns=0;i=54593",
                    object_id: "ns=0;i=54593",
                    inputArguments: []
                },
                {
                    ocb_id: "Temperature",
                    opcua_id: "ns=0;i=54596",
                    object_id: "ns=0;i=54596",
                    inputArguments: []
                },
                {
                    ocb_id: "Velocity",
                    opcua_id: "ns=0;i=54594",
                    object_id: "ns=0;i=54594",
                    inputArguments: []
                },
            ]
        },
        // {
        //     id: "urn:ngsi-ld:Robot:ur5e",
        //     type: "Robot",
        //     joint: {type: "Relationship",
        //             object: "urn:ngsi-ld:JointType:base"},

        //     mappings: []
        // },
    ],

    contextSubscriptions: [
       
        {
            id: "urn:ngsi-ld:JointType:base",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "base",
                    opcua_id: "ns=0;i=54562",
                    object_id: "ns=0;i=54562",
                    inputArguments: [
                        {}
                    ]
                },
            ]
        },

        {
            id: "urn:ngsi-ld:JointType:elbow",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "elbow",
                    opcua_id: "ns=0;i=54574",
                    object_id: "ns=0;i=54574",
                    inputArguments: [
                        {}
                    ]
                },
            ]
        },

        {
            id: "urn:ngsi-ld:JointType:shoulder",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "shoulder",
                    opcua_id: "ns=0;i=54568",
                    object_id: "ns=0;i=54568",
                    inputArguments: [
                        {}
                    ]
                },
            ]
        },

        {
            id: "urn:ngsi-ld:JointType:wrist1",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "wrist1",
                    opcua_id: "ns=0;i=54580",
                    object_id: "ns=0;i=54580",
                    inputArguments: [
                        {}
                    ]
                },
            ]
        },

        {
            id: "urn:ngsi-ld:JointType:wrist2",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "wrist2",
                    opcua_id: "ns=0;i=54586",
                    object_id: "ns=0;i=54586",
                    inputArguments: [
                        {}
                    ]
                },
            ]
        },

        {
            id: "urn:ngsi-ld:JointType:wrist3",
            type: "JointType",
            mappings: [
                {
                    ocb_id: "wrist3",
                    opcua_id: "ns=0;i=54592",
                    object_id: "ns=0;i=54592",
                    inputArguments: [
                        {}
                    ]
                },
            ]
        },

        {
            id: "urn:ngsi-ld:TcpType:tcp0",
            type: "TcpType",
            mappings: [
                {
                    ocb_id: "tcp0",
                    opcua_id: "ns=0;i=54598",
                    object_id: "ns=0;i=54598",
                    inputArguments: [
                        {}
                    ]
                },
                
            ]
        },
       

    ],
    events: [],
    /**
     * Default service, for IoT Agent installations that won't require preregistration.
     */
    service: 'opcua_ur5e',
    /**
     * Default subservice, for IoT Agent installations that won't require preregistration.
     */
    subservice: '/mydemo',
    /**
     * URL Where the IoT Agent Will listen for incoming updateContext and queryContext requests (for commands and
     * passive attributes). This URL will be sent in the Context Registration requests.
     */
    providerUrl: 'http://iot_agent_robot:4041',
    /**
     * Default maximum expire date for device registrations.
     */
    deviceRegistrationDuration: 'P20Y',
    /**
     * Default type, for IoT Agent installations that won't require preregistration.
     */
    defaultType: 'Device',
    /**
     * Default resource of the IoT Agent. This value must be different for every IoT Agent connecting to the IoT
     * Manager.
     */
    defaultResource: '/iot/opcua',
    /**
     * Flag indicating whether the incoming measures to the IoTAgent should be processed as per the "attributes" field.
     */
    explicitAttrs: false,
    /**
     * List of characters to be filtered before forwarding any request to Orion.
     * Default Orion forbidden characters are filtered by default, see (https://github.com/telefonicaid/fiware-orion/blob/74aaae0c98fb24f082e3b258aa642461eb285e39/doc/manuals/orion-api.md#general-syntax-restrictions)
     */
    extendedForbiddenCharacters: [],
    /**
     * Flag indicating whether to provision the Group and Device automatically
     */
    autoprovision: true,
    /**
     * Default limit for express router built into iotagent-node-lib module
     */
    expressLimit: '50mb'
};

config.opcua = {
    /**
     * Subscription options for OPC UA connection.
     */
    subscription: {
        maxNotificationsPerPublish: 1000,
        publishingEnabled: true,
        requestedLifetimeCount: 100,
        requestedMaxKeepAliveCount: 10,
        requestedPublishingInterval: 1000,
        priority: 128
    },
    /**
     * Endpoint where the IoT Agent will listen for an active OPC UA Server.
     */
    endpoint: 'opc.tcp://host.docker.internal:4840',
    /**
     * Security Mode to access OPC UA Server.
     */
    securityMode: 'None',
    /**
     * Security Policy to access OPC UA Server.
     */
    securityPolicy: 'None',
    /**
     * Username to access OPC UA Server.
     */
    username: null,
    /**
     * Password to access OPC UA Server.
     */
    password: null,
    /**
     * Flag indicating whether the OPC uA variables readings should be handled as single subscription.
     */
    uniqueSubscription: false
};

config.mappingTool = {
    /**
     *  Boolean property to assess whether enabling polling in MappingTool or not
     */
    polling: false,
    /**
     * agentId prefix to be assigned to the newly generated entity from MappingTool execution
     */
    agentId: 'agee01_',
    /**
     * Namespaces to ignore when crawling nodes from OPC UA Server
     */
    namespaceIgnore: '7',
    /**
     * entityId to be assigned to the newly generated entity from MappingTool execution
     */
    entityId: 'agee01_mos',
    /**
     * entityType to be assigned to the newly generated entity from MappingTool execution
     */
    entityType: 'Device',
    /**
     * boolean flag to determine whether to store the output of the mapping tool execution or not
     */
    storeOutput: true
};

/**
 * Flag indicating which configuration type to perform. Possible choices are:
 *  - auto : mappingTool will be run and runtime device mappings will be loaded
 *  - dynamic : device mappings from config.js will be ignored, REST API Provisioning is mandatory
 *  - static : device mappings from config.js will be loaded
 */

config.configurationType = 'static';
/**
 * map {name: function} of extra transformations avaliable at JEXL plugin
 *  see https://github.com/telefonicaid/iotagent-node-lib/tree/master/doc/expressionLanguage.md#available-functions
 */

config.jexlTransformations = {};

/**
 * Flag indicating whether the incoming notifications to the IoTAgent should be processed using the bidirectionality
 * plugin from the latest versions of the library or the OPCUA-specific configuration retrieval mechanism.
 */
config.configRetrieval = false;
/**
 * Default API Key, to use with device that have been provisioned without a Configuration Group.
 */
config.defaultKey = 'iot';
/**
 * Default transport protocol when no transport is provisioned through the Device Provisioning API.
 */
config.defaultTransport = 'OPCUA';
/**
 * Flag indicating whether the node server will be executed in multi-core option (true) or it will be a
 * single-thread one (false).
 */
//config.multiCore = false;

module.exports = config;